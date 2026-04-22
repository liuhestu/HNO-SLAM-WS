#include "Initializer.h"

#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

namespace hno_slam {

void Initializer::feed_imu(const ov_core::ImuData& msg) {
  imu_buffer_.push_back(msg);
  if (imu_buffer_.size() > window_size_) {
    imu_buffer_.erase(imu_buffer_.begin());
  }
}

bool Initializer::initialize(State& state,
                             Eigen::Vector3d& bg_gyro,
                             Eigen::Vector3d& ba_accel,
                             double& timestamp) {
  if (imu_buffer_.size() < window_size_) {
    return false;
  }

  Eigen::Vector3d ave_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d ave_gyro = Eigen::Vector3d::Zero();

  for (const auto& m : imu_buffer_) {
    ave_acc += m.am;
    ave_gyro += m.wm;
  }

  ave_acc /= static_cast<double>(imu_buffer_.size());
  ave_gyro /= static_cast<double>(imu_buffer_.size());

  double acc_var = 0.0;
  double gyro_var = 0.0;
  for (const auto& m : imu_buffer_) {
    acc_var += (m.am - ave_acc).squaredNorm();
    gyro_var += (m.wm - ave_gyro).squaredNorm();
  }
  acc_var /= static_cast<double>(imu_buffer_.size());
  gyro_var /= static_cast<double>(imu_buffer_.size());

  if (acc_var > max_acc_variance_ || gyro_var > max_gyro_variance_) {
    imu_buffer_.clear();
    return false;
  }

  bg_gyro = ave_gyro;
  ba_accel.setZero();

  const Eigen::Vector3d acc_dir = ave_acc.normalized();
  const Eigen::Quaterniond R0_q = Eigen::Quaterniond::FromTwoVectors(acc_dir, Eigen::Vector3d::UnitZ());

  state.R_hat = R0_q.toRotationMatrix();
  state.p_hat.setZero();
  state.v_hat.setZero();
  state.g_hat = Eigen::Vector3d(0.0, 0.0, -9.81);

  timestamp = imu_buffer_.back().timestamp;

  const double roll = std::atan2(state.R_hat(2, 1), state.R_hat(2, 2));
  const double pitch = std::asin(-state.R_hat(2, 0));
  const double yaw = std::atan2(state.R_hat(1, 0), state.R_hat(0, 0));
  std::cout << "[Initializer] success. RPY(rad)= " << roll << ", " << pitch << ", " << yaw
            << " | bg_gyro= " << bg_gyro.transpose() << std::endl;

  imu_buffer_.clear();
  return true;
}

}  // namespace hno_slam
