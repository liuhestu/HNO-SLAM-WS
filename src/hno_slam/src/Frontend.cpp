#include "Frontend.h"

#include <unordered_set>

#include <opencv2/calib3d.hpp>

namespace hno_slam {

Frontend::Frontend(const std::vector<std::shared_ptr<ov_core::CamBase>>& cameras,
                   const std::vector<Eigen::Matrix4d>& T_C_B)
    : cameras_(cameras), T_C_B_(T_C_B) {
  // 将相机列表转成 TrackKLT 需要的 id->相机模型映射。
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cam_map;
  for (size_t i = 0; i < cameras_.size(); ++i) {
    cam_map[i] = cameras_[i];
  }

  // 创建 KLT 前端：
  // - 200: 最大特征数量
  // - 0: 不使用金字塔层级偏置
  // - true + HISTOGRAM: 启用直方图均衡，增强弱纹理场景稳定性
  // - 其余参数控制网格分布与跟踪窗口，沿用当前工程经验值
  tracker_ = std::make_shared<ov_core::TrackKLT>(cam_map,
                                                  200,
                                                  0,
                                                  true,
                                                  ov_core::TrackBase::HISTOGRAM,
                                                  20,
                                                  5,
                                                  5,
                                                  15);
}

std::vector<FeatureObs> Frontend::process_frame(const ov_core::CameraData& msg) {
  std::vector<FeatureObs> output;

  // 缓存最近一帧双目图像，供可视化兜底使用（当 display_history 没返回图时）。
  if (msg.images.size() >= 2 && !msg.images[0].empty() && !msg.images[1].empty()) {
    last_left_img_ = msg.images[0].clone();
    last_right_img_ = msg.images[1].clone();
    last_sensor_ids_ = msg.sensor_ids;
    has_last_stereo_ = true;
  }

  // 前置条件检查：
  // tracker_ 必须可用；双目相机/外参数量必须满足；当前消息也必须至少包含两个 sensor id。
  if (!tracker_ || cameras_.size() < 2 || T_C_B_.size() < 2 || msg.sensor_ids.size() < 2) {
    return output;
  }

  // 喂入新相机数据并读取当前帧跟踪结果（按相机 id 组织）。
  tracker_->feed_new_camera(msg);
  const auto obs_raw = tracker_->get_last_obs();
  const auto ids_raw = tracker_->get_last_ids();

  // 当前约定：sensor_ids[0] 是左目，sensor_ids[1] 是右目。
  const size_t left_cam_id = static_cast<size_t>(msg.sensor_ids.at(0));
  const size_t right_cam_id = static_cast<size_t>(msg.sensor_ids.at(1));

  // 若左目本帧无可用观测，直接返回空（后端本帧不更新视觉量测）。
  auto left_obs_it = obs_raw.find(left_cam_id);
  auto left_ids_it = ids_raw.find(left_cam_id);
  if (left_obs_it == obs_raw.end() || left_ids_it == ids_raw.end() || left_obs_it->second.empty()) {
    return output;
  }

  // 左目关键点与其 feature id（一一对应）。
  const auto& left_kps = left_obs_it->second;
  const auto& left_ids = left_ids_it->second;

  // 记录本帧被判定为外点的 feature id。
  std::unordered_set<size_t> outlier_ids;
  {
    // 时序几何一致性检查：
    // 使用“上一帧左目位置 -> 当前帧左目位置”做基础矩阵 RANSAC，剔除错配/动态外点。
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> curr_pts;
    std::vector<size_t> ransac_ids;
    prev_pts.reserve(left_ids.size());
    curr_pts.reserve(left_ids.size());
    ransac_ids.reserve(left_ids.size());

    for (size_t i = 0; i < left_ids.size(); ++i) {
      // 仅使用有历史观测的点参与时序 RANSAC。
      auto hit = history_obs_.find(left_ids[i]);
      if (hit == history_obs_.end()) {
        continue;
      }
      prev_pts.push_back(hit->second);
      curr_pts.push_back(left_kps[i].pt);
      ransac_ids.push_back(left_ids[i]);
    }

    if (prev_pts.size() >= 8) {
      // 点数足够时才估计 F；阈值 2.0 像素，置信度 0.99。
      std::vector<uchar> status;
      cv::findFundamentalMat(prev_pts, curr_pts, cv::FM_RANSAC, 2.0, 0.99, status);
      for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i]) {
          // status=0 的 id 标记为外点，后续直接跳过。
          outlier_ids.insert(ransac_ids[i]);
        }
      }
    }
  }

  // 右目建立 id->索引哈希表，便于 O(1) 查询双目对应。
  std::unordered_map<size_t, size_t> right_cam_idx;
  auto right_obs_it = obs_raw.find(right_cam_id);
  auto right_ids_it = ids_raw.find(right_cam_id);
  if (right_obs_it != obs_raw.end() && right_ids_it != ids_raw.end()) {
    const auto& right_ids = right_ids_it->second;
    right_cam_idx.reserve(right_ids.size());
    for (size_t i = 0; i < right_ids.size(); ++i) {
      right_cam_idx[right_ids[i]] = i;
    }
  }

  // 当前帧用于下一帧 RANSAC 的“左目历史观测表”。
  std::map<size_t, cv::Point2f> current_history;

  for (size_t i = 0; i < left_ids.size(); ++i) {
    const size_t id = left_ids[i];
    // 1) 跳过 RANSAC 判定外点。
    if (outlier_ids.find(id) != outlier_ids.end()) {
      continue;
    }

    // 2) 必须在右目找到同 id 才能形成双目观测。
    auto right_idx_it = right_cam_idx.find(id);
    if (right_idx_it == right_cam_idx.end() || right_obs_it == obs_raw.end()) {
      continue;
    }

    const cv::Point2f& pt_l = left_kps[i].pt;
    const cv::Point2f& pt_r = right_obs_it->second[right_idx_it->second].pt;

    // 3) 像素坐标去畸变并转归一化平面坐标。
    //    注意：前端只输出 2D 观测，不负责 3D 初始化与地图生命周期。
    Eigen::Vector2d uv_l_norm = cameras_[0]->undistort_d(Eigen::Vector2d(pt_l.x, pt_l.y));
    Eigen::Vector2d uv_r_norm = cameras_[1]->undistort_d(Eigen::Vector2d(pt_r.x, pt_r.y));

    // 4) 组织成后端更新所需的最小观测结构。
    FeatureObs obs;
    obs.id = id;
    obs.uv_l_norm = uv_l_norm;
    obs.uv_r_norm = uv_r_norm;

    output.push_back(obs);
    // 保存当前左目像素位置，作为下一帧时序 RANSAC 的 prev 点。
    current_history[id] = pt_l;
  }

  // 用当前帧历史替换旧历史（仅保留当前仍有效的 id）。
  history_obs_ = std::move(current_history);
  return output;
}

bool Frontend::render_track_overlay(cv::Mat& img_out,
                                    const std::unordered_set<size_t>& active_ids) {
  // 没有 tracker 无法绘制。
  if (!tracker_) {
    return false;
  }

  // 先绘制轨迹历史：传入 active_ids 让历史绘制和后端成熟点语义对齐。
  tracker_->display_history(img_out, 0, 255, 0, 255, 0, 0, {}, "HNO Tracker", &active_ids);
  // 若历史图为空，则用缓存的最新双目灰度图拼接为 BGR 画布作为兜底。
  if (img_out.empty() && has_last_stereo_ && !last_left_img_.empty() && !last_right_img_.empty()) {
    cv::Mat left_bgr;
    cv::Mat right_bgr;
    cv::cvtColor(last_left_img_, left_bgr, cv::COLOR_GRAY2BGR);
    cv::cvtColor(last_right_img_, right_bgr, cv::COLOR_GRAY2BGR);
    cv::hconcat(left_bgr, right_bgr, img_out);
  }

  // 仍为空说明当前没有可渲染内容。
  if (img_out.empty()) {
    return false;
  }

  // 获取 tracker 最近一帧观测与 id，用于叠加当前帧圆圈。
  const auto obs = tracker_->get_last_obs();
  const auto ids = tracker_->get_last_ids();
  if (obs.empty() || ids.empty()) {
    // 仅有历史轨迹图也算成功。
    return true;
  }

  // 优先使用最近消息里的传感器 id；若不可用则按 0/1 退化。
  const size_t left_cam_id = (last_sensor_ids_.size() >= 2) ? static_cast<size_t>(last_sensor_ids_[0]) : 0;
  const size_t right_cam_id = (last_sensor_ids_.size() >= 2) ? static_cast<size_t>(last_sensor_ids_[1]) : 1;

  // 判断左右相机观测是否可用；单目可用时按单图宽度处理。
  const bool has_cam0 = obs.count(left_cam_id) > 0 && ids.count(left_cam_id) > 0;
  const bool has_cam1 = obs.count(right_cam_id) > 0 && ids.count(right_cam_id) > 0;
  const int num_cams = has_cam1 ? 2 : 1;
  const int width = img_out.cols / num_cams;
  const int height = img_out.rows;
  if (width <= 0 || height <= 0) {
    return true;
  }

  if (has_cam0 && img_out.cols >= width && img_out.rows >= height) {
    cv::Mat left_roi = img_out(cv::Rect(0, 0, width, height));
    const size_t n = std::min(obs.at(left_cam_id).size(), ids.at(left_cam_id).size());
    for (size_t i = 0; i < n; ++i) {
      const size_t id = ids.at(left_cam_id)[i];
      const cv::Point2f pt = obs.at(left_cam_id)[i].pt;
      // 左图当前帧叠加语义：红圈=后端 active（成熟点），黄圈=仅跟踪未入状态。
      if (active_ids.count(id) > 0) {
        cv::circle(left_roi, pt, 6, cv::Scalar(0, 0, 255), 2);
      } else {
        cv::circle(left_roi, pt, 6, cv::Scalar(0, 255, 255), 1);
      }
    }
  }

  if (has_cam1 && img_out.cols >= 2 * width && img_out.rows >= height) {
    cv::Mat right_roi = img_out(cv::Rect(width, 0, width, height));
    const size_t n = std::min(obs.at(right_cam_id).size(), ids.at(right_cam_id).size());
    for (size_t i = 0; i < n; ++i) {
      const size_t id = ids.at(right_cam_id)[i];
      const cv::Point2f pt = obs.at(right_cam_id)[i].pt;
      // 右图使用与左图一致的红/黄语义，便于双目对照观察。
      if (active_ids.count(id) > 0) {
        cv::circle(right_roi, pt, 6, cv::Scalar(0, 0, 255), 2);
      } else {
        cv::circle(right_roi, pt, 6, cv::Scalar(0, 255, 255), 1);
      }
    }
  }

  return true;
}
}  // namespace hno_slam
