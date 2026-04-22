#pragma once

#include <map>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include "../thirdparty/cam/CamBase.h"
#include "../thirdparty/track/TrackKLT.h"
#include "../thirdparty/utils/sensor_data.h"

namespace hno_slam {

struct FeatureObs {
    size_t id = 0;
    Eigen::Vector2d uv_l_norm = Eigen::Vector2d::Zero();
    Eigen::Vector2d uv_r_norm = Eigen::Vector2d::Zero();
};

class Frontend {
 public:
    Frontend(const std::vector<std::shared_ptr<ov_core::CamBase>>& cameras,
                     const std::vector<Eigen::Matrix4d>& T_C_B);

    std::vector<FeatureObs> process_frame(const ov_core::CameraData& msg);

    bool render_track_overlay(cv::Mat& img_out,
                              const std::unordered_set<size_t>& active_ids);

 private:
    std::shared_ptr<ov_core::TrackKLT> tracker_;
    std::vector<std::shared_ptr<ov_core::CamBase>> cameras_;
    std::vector<Eigen::Matrix4d> T_C_B_;
    std::map<size_t, cv::Point2f> history_obs_;

    std::vector<int> last_sensor_ids_;
    cv::Mat last_left_img_;
    cv::Mat last_right_img_;
    bool has_last_stereo_ = false;
};

}  // namespace hno_slam
