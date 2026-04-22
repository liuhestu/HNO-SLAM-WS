#ifndef HNO_FEATURE_H
#define HNO_FEATURE_H

#include <memory>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include "track/TrackKLT.h"
#include "cam/CamBase.h"
#include "HNOUpdater.h"
#include <opencv2/core/types.hpp>

namespace ov_hno {

// 内部管理的特征点结构（简化版：直接维护当前估计的世界坐标）
struct FeatureInfo {
    Eigen::Vector3d p_w;     // 世界系坐标（基于最近一次可信双目观测）
    int track_count;         // 成功跟踪帧计数
    int fail_count;          // 连续失败计数
};

class HNOFeature {
public:
    HNOFeature(std::vector<std::shared_ptr<ov_core::CamBase>> cams,
               std::vector<Eigen::Matrix4d> T_C_B);

    // 核心处理函数
    void feed_measurement(const ov_core::CameraData& message, 
                          Eigen::Matrix3d R_est, Eigen::Vector3d p_est,
                          Eigen::Matrix3d R_gt,  Eigen::Vector3d p_gt, // 兼容接口，GT仅可选
                          std::vector<HNOObservation>& observations);

    const std::map<size_t, Eigen::Vector3d> get_active_map() const;
    std::shared_ptr<ov_core::TrackKLT> get_tracker() { return tracker; }

private:
    std::shared_ptr<ov_core::TrackKLT> tracker;
    std::vector<std::shared_ptr<ov_core::CamBase>> cameras;
    std::vector<Eigen::Matrix4d> T_C_B; // Cam to Body

    // 核心数据库: ID -> FeatureInfo
    std::map<size_t, FeatureInfo> feature_db;
    
    // RANSAC 辅助
    std::map<size_t, cv::Point2f> history_obs; 

    // 双目三角化 (返回相机系坐标)
    bool triangulate_stereo(const Eigen::Vector3d& uv_left, 
                           const Eigen::Vector3d& uv_right, 
                           Eigen::Vector3d& p_c_left,
                           double* reproj_err_right = nullptr);
                           
    // 将点投影并检查误差
    bool check_reprojection(size_t id, const Eigen::Vector3d& p_w, 
                            const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb,
                            const Eigen::Vector3d& uv_meas_norm,
                            double reproj_thresh,
                            double* reproj_err = nullptr);
};

}
#endif