#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "Frontend.h"
#include "Initializer.h"
#include "Observer.h"
#include "State.h"
#include "../thirdparty/utils/sensor_data.h"

namespace hno_slam {

class System {
 public:
  explicit System(const std::string& config_path);

  void feed_imu(const ov_core::ImuData& msg);
  void feed_camera(const ov_core::CameraData& msg);
  std::shared_ptr<State> get_state();
  bool get_track_visualization(cv::Mat& img);

 private:
  ov_core::ImuData interpolate_imu(const ov_core::ImuData& a,
                                   const ov_core::ImuData& b,
                                   double t) const;
  void calibrate_imu(const ov_core::ImuData& raw,
                     const Eigen::Vector3d& bg,
                     const Eigen::Vector3d& ba,
                     Eigen::Vector3d& omega_b,
                     Eigen::Vector3d& acc_b) const;

  std::shared_ptr<State> state_;
  std::shared_ptr<Observer> observer_;
  std::shared_ptr<Frontend> frontend_;
  std::shared_ptr<Initializer> initializer_;

  std::mutex data_mutex_;
  std::deque<ov_core::ImuData> imu_buf_;
  bool is_initialized_;
  double last_img_time_;
  Eigen::Vector3d bg_gyro_;
  Eigen::Vector3d ba_accel_;

  Eigen::Matrix3d Tw_;
  Eigen::Matrix3d Ta_;
  Eigen::Matrix3d R_imu_to_gyro_;
  Eigen::Matrix3d R_imu_to_acc_;
  Eigen::Matrix3d Tg_;
  Eigen::Matrix4d T_I_B_;
  Eigen::Matrix3d R_B_I_;

  cv::Mat last_track_viz_;
  bool has_track_viz_;
};

}  // namespace hno_slam
