#include "HNOFeature.h"
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <unordered_set>

using namespace ov_hno;

HNOFeature::HNOFeature(std::vector<std::shared_ptr<ov_core::CamBase>> cams,
                       std::vector<Eigen::Matrix4d> extrinsics) 
    : cameras(cams), T_C_B(extrinsics) {
    
    std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cam_map;
    for(size_t i=0; i<cams.size(); ++i) cam_map[i] = cams[i];
    
    // 稍微增加一点点特征点上限，因为我们会扔掉很多不稳定的点
    tracker = std::make_shared<ov_core::TrackKLT>(cam_map, 200, 0, true, 
              ov_core::TrackBase::HistogramMethod::HISTOGRAM, 20, 5, 5, 15);
}

void HNOFeature::feed_measurement(const ov_core::CameraData& message, 
                                  Eigen::Matrix3d R_est, Eigen::Vector3d p_est,
                                  Eigen::Matrix3d R_gt,  Eigen::Vector3d p_gt,
                                  std::vector<HNOObservation>& observations) {
    static int frame_counter = 0;
    frame_counter++;

    tracker->feed_new_camera(message);

    auto obs_raw = tracker->get_last_obs();
    auto ids_raw = tracker->get_last_ids();

    if(!obs_raw.count(0)) return; 
    if(obs_raw[0].empty()) return; // 无特征时直接退出，避免尾帧继续估计

    // --- 1. RANSAC (2D-2D) 剔除动态点 ---
    std::unordered_set<size_t> outlier_ids;
    if (!history_obs.empty()) {
        std::vector<cv::Point2f> pts_prev, pts_curr;
        std::vector<size_t> pts_ids;
        for(size_t i=0; i<ids_raw[0].size(); ++i) {
            size_t id = ids_raw[0][i];
            if(history_obs.count(id)) {
                pts_prev.push_back(history_obs[id]);
                pts_curr.push_back(obs_raw[0][i].pt);
                pts_ids.push_back(id);
            }
        }
        if(pts_prev.size() >= 15) {
             std::vector<uchar> status;
             cv::findFundamentalMat(pts_prev, pts_curr, cv::FM_RANSAC, 2.0, 0.99, status);
             for(size_t k=0; k<status.size(); ++k) {
                 if(!status[k]) outlier_ids.insert(pts_ids[k]);
             }
        }
    }

    // --- 准备数据 ---
    observations.clear();
    std::vector<size_t> current_frame_ids;
    std::map<size_t, cv::Point2f> next_history_obs;

    std::map<size_t, int> right_cam_idx;
    if(obs_raw.count(1)) {
        for(size_t i=0; i<ids_raw[1].size(); ++i) right_cam_idx[ids_raw[1][i]] = i;
    }

    // 获取外参 (Body -> World)
    // 这里传入的 R_gt / p_gt 在 Cheat 模式下是 GT，否则是估计值
    Eigen::Matrix3d R_wb = R_gt;
    Eigen::Vector3d p_wb = p_gt;
    
    // 获取内参 T_bc (Cam -> Body)
    Eigen::Matrix3d R_bc = T_C_B[0].block<3,3>(0,0);
    Eigen::Vector3d p_bc = T_C_B[0].block<3,1>(0,3);

    int count_stable = 0;
    int count_new = 0;
    int reproj_pass = 0, reproj_reject = 0;
    double reproj_err_max = 0.0, reproj_err_sum = 0.0;
    int stereo_pass = 0, stereo_reject = 0;
    double stereo_err_max = 0.0;

    bool low_feat_mode = false; // 自适应：少点时放宽门限并提前输出

    // --- 遍历左目所有特征点 ---
    size_t num_pts = obs_raw[0].size();
    if(num_pts < 80 || feature_db.size() < 60) low_feat_mode = true;
    double reproj_thresh = low_feat_mode ? 0.10 : 0.08;
    int mature_thresh = low_feat_mode ? 2 : 3;
    for(size_t i=0; i<num_pts; ++i) {
        size_t id = ids_raw[0][i];
        if(outlier_ids.count(id)) continue; // 跳过 2D RANSAC 外点

        current_frame_ids.push_back(id);
        next_history_obs[id] = obs_raw[0][i].pt;

        // 计算左目归一化坐标
        Eigen::Vector2d uv_l_px(obs_raw[0][i].pt.x, obs_raw[0][i].pt.y);
        Eigen::Vector2d uv_l_norm = cameras[0]->undistort_d(uv_l_px); 
        Eigen::Vector3d uv_l_vec(uv_l_norm.x(), uv_l_norm.y(), 1.0); 

        // 检查右目是否存在
        bool has_right = false;
        Eigen::Vector3d uv_r_vec = Eigen::Vector3d::Zero();
        if(right_cam_idx.count(id)) {
            int idx_r = right_cam_idx[id];
            cv::Point2f pt_r = obs_raw[1][idx_r].pt;
            Eigen::Vector2d uv_r_px(pt_r.x, pt_r.y);
            Eigen::Vector2d uv_r_norm = cameras[1]->undistort_d(uv_r_px);
            uv_r_vec << uv_r_norm.x(), uv_r_norm.y(), 1.0;
            has_right = true;
        }

        // --- 逻辑分支 ---
        // 1. 老点：使用当前帧双目刷新 p_w；若无双目则重投影检查
        if(feature_db.count(id)) {
            FeatureInfo& info = feature_db[id];

            bool stereo_ok = false;
            Eigen::Vector3d p_w_new = info.p_w;
            if(has_right) {
                Eigen::Vector3d p_c_curr;
                double stereo_err = 0.0;
                if(triangulate_stereo(uv_l_vec, uv_r_vec, p_c_curr, &stereo_err)) {
                    double depth = p_c_curr.z();
                    if(depth > 0.5 && depth < 10.0) {
                        stereo_pass++;
                        if(stereo_err > stereo_err_max) stereo_err_max = stereo_err;
                        stereo_ok = true;
                        Eigen::Vector3d p_body = R_bc * p_c_curr + p_bc;
                        p_w_new = R_wb * p_body + p_wb;

                        double dist = (info.p_w - p_w_new).norm();
                        if(dist > 0.5) {
                            // 大跳变当作误匹配，直接移除
                            feature_db.erase(id);
                            continue;
                        } else {
                            double alpha = 1.0 / (info.track_count + 1.0);
                            if(alpha < 0.05) alpha = 0.05;
                            if(alpha > 0.2) alpha = 0.2;
                            info.p_w = (1.0 - alpha) * info.p_w + alpha * p_w_new;
                        }
                    } else {
                        stereo_reject++;
                    }
                }
                else {
                    stereo_reject++;
                }
            }

            Eigen::Vector3d p_w_est = stereo_ok ? p_w_new : info.p_w;
            double reproj_err = 0.0;
            bool reproj_ok = stereo_ok ? true : check_reprojection(id, p_w_est, R_wb, p_wb, uv_l_vec, reproj_thresh, &reproj_err);

            if(!reproj_ok) {
                info.fail_count++;
                if(info.fail_count > 3) { feature_db.erase(id); }
                reproj_reject++;
                continue;
            }

            reproj_pass++;
            reproj_err_sum += reproj_err;
            if(reproj_err > reproj_err_max) reproj_err_max = reproj_err;

            info.fail_count = 0;
            if(info.track_count < 15) info.track_count++;

            // 只有成熟点才输出观测；少点时提前输出
            if(info.track_count >= mature_thresh) {
                HNOObservation obs;
                obs.uv_left = uv_l_vec.normalized();
                if(stereo_ok) {
                    obs.uv_right = uv_r_vec.normalized();
                    obs.isValidRight = true;
                } else {
                    obs.isValidRight = false;
                }
                obs.xyz = info.p_w;
                observations.push_back(obs);
                count_stable++;
            }

        }
        // 2. 新点：双目初始化，先建图不输出，等待成熟
        else if(has_right) {
            Eigen::Vector3d p_c_left;
            double stereo_err = 0.0;
            if(triangulate_stereo(uv_l_vec, uv_r_vec, p_c_left, &stereo_err)) {
                double depth = p_c_left.z();
                if(depth > 0.5 && depth < 10.0) {
                    stereo_pass++;
                    if(stereo_err > stereo_err_max) stereo_err_max = stereo_err;
                    FeatureInfo info;
                    Eigen::Vector3d p_body = R_bc * p_c_left + p_bc;
                    info.p_w = R_wb * p_body + p_wb;
                    info.track_count = 1;
                    info.fail_count = 0;
                    feature_db[id] = info;
                    count_new++;
                } else {
                    stereo_reject++;
                }
            } else {
                stereo_reject++;
            }
        }
    } 

    // 4. 地图清理
    std::unordered_set<size_t> klt_alive_ids;
    if(obs_raw.count(0)) {
        for(size_t id : ids_raw[0]) klt_alive_ids.insert(id);
    }
    
    int fail_limit = low_feat_mode ? 8 : 5;

    for (auto it = feature_db.begin(); it != feature_db.end(); ) {
        bool alive = klt_alive_ids.find(it->first) != klt_alive_ids.end();
        bool too_many_fail = it->second.fail_count > fail_limit;

        if(!alive || too_many_fail) {
            it = feature_db.erase(it);
        } else {
            ++it;
        }
    }
    
    history_obs = next_history_obs;

    // 节流日志：每 30 帧输出一次前端健康度
    if(frame_counter % 30 == 0) {
        int reproj_total = reproj_pass + reproj_reject;
        int stereo_total = stereo_pass + stereo_reject;
        double reproj_mean = reproj_pass > 0 ? reproj_err_sum / reproj_pass : 0.0;
        std::cout << std::fixed << std::setprecision(3)
                  << "[HNOFeature] frame " << frame_counter
                  << " pts " << num_pts
                  << " stable " << count_stable
                  << " new " << count_new
                  << " db " << feature_db.size()
                  << " stereo " << stereo_pass << "/" << stereo_total
                  << " err_max " << stereo_err_max
                  << " reproj " << reproj_pass << "/" << reproj_total
                  << " err_mean " << reproj_mean
                  << " err_max " << reproj_err_max
                  << std::endl;
    }
}

// 检查重投影误差
// 如果返回 false，说明点有问题
bool HNOFeature::check_reprojection(size_t id, const Eigen::Vector3d& p_w, 
                                    const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb,
                                    const Eigen::Vector3d& uv_meas_norm,
                                    double reproj_thresh,
                                    double* reproj_err) {
    
    // 1. 转到 Body 系: P_b = R_wb^T * (P_w - p_wb)
    Eigen::Vector3d p_b = R_wb.transpose() * (p_w - p_wb);
    
    // 2. 转到 Cam 系: P_c = R_bc^T * (P_b - p_bc)
    Eigen::Matrix3d R_bc = T_C_B[0].block<3,3>(0,0);
    Eigen::Vector3d p_bc = T_C_B[0].block<3,1>(0,3);
    Eigen::Vector3d p_c = R_bc.transpose() * (p_b - p_bc);

    // 3. 深度检查 (必须在相机前方)
    if(p_c.z() < 0.2) return false;

    // 4. 投影到归一化平面
    Eigen::Vector2d uv_proj = p_c.head<2>() / p_c.z();
    Eigen::Vector2d uv_meas = uv_meas_norm.head<2>(); 

    // 5. 计算误差
    double err = (uv_proj - uv_meas).norm();
    if(reproj_err) *reproj_err = err;

    // 自适应阈值（正常 0.08，少点模式 0.10）
    if(err > reproj_thresh) return false;

    return true;
}

bool HNOFeature::triangulate_stereo(const Eigen::Vector3d& uv_left, 
                                   const Eigen::Vector3d& uv_right, 
                                   Eigen::Vector3d& p_c_left,
                                   double* reproj_err_right) {
    // 保持严格三角化逻辑
    Eigen::Matrix4d T_Right_Left = T_C_B[1].inverse() * T_C_B[0];
    Eigen::Matrix3d R = T_Right_Left.block<3,3>(0,0);
    Eigen::Vector3d t = T_Right_Left.block<3,1>(0,3);

    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = R * uv_left;
    A.col(1) = -uv_right;
    Eigen::Vector3d b = -t;
    Eigen::Vector2d x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    double d1 = x(0);

    // 严格检查
    if(d1 < 0.5 || d1 > 15.0) return false; 
    
    p_c_left = uv_left * d1;
    
    // 重投影检查
    Eigen::Vector3d P_R = R * p_c_left + t;
    Eigen::Vector2d uv_r_proj = (P_R / P_R(2)).head<2>();
    double err = (uv_r_proj - uv_right.head<2>()).norm();
    if(reproj_err_right) *reproj_err_right = err;
    
    if(err > 0.015) return false; // 略收紧双目匹配误差，提升深度质量

    return true;
}

const std::map<size_t, Eigen::Vector3d> HNOFeature::get_active_map() const {
    std::map<size_t, Eigen::Vector3d> out;
    for(const auto& pair : feature_db) {
        const FeatureInfo& info = pair.second;
        if(info.track_count >= 3) {
            out[pair.first] = info.p_w;
        }
    }
    return out;
}