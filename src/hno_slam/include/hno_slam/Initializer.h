#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "../thirdparty/utils/sensor_data.h"
#include "State.h"

namespace hno_slam {

class Initializer {
 public:
  void feed_imu(const ov_core::ImuData& msg);
  bool initialize(State& state, Eigen::Vector3d& bg_gyro, Eigen::Vector3d& ba_accel, double& timestamp);

  std::vector<ov_core::ImuData> imu_buffer_;
  size_t window_size_ = 200;
  double max_acc_variance_ = 0.05;
  double max_gyro_variance_ = 0.01;
};

}  // namespace hno_slam
