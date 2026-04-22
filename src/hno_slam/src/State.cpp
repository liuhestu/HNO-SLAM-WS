#include "State.h"

#include <iostream>

namespace hno_slam {

State::State() {
  R_hat.setIdentity();
  p_hat.setZero();
  v_hat.setZero();
  g_hat.setZero();

  for (int i = 0; i < N_MAX; ++i) {
    p_L[i].setZero();
    is_active[i] = false;
    feature_ids[i] = 0;
    track_counts[i] = 0;
  }

  const int total_dim = 6 + 3 * N_MAX;
  P = Eigen::MatrixXd::Zero(total_dim, total_dim);
  P.diagonal().setConstant(1e-8);
}

int State::add_landmark(size_t id, const Eigen::Vector3d& p_w_init, double initial_variance) {
  int idx = -1;
  for (int i = 0; i < N_MAX; ++i) {
    if (!is_active[i]) {
      idx = i;
      break;
    }
  }

  if (idx < 0) {
    std::cerr << "[State] Warning: landmark pool is full, cannot add id=" << id << std::endl;
    return -1;
  }

  is_active[idx] = true;
  feature_ids[idx] = id;
  track_counts[idx] = 0;
  p_L[idx] = p_w_init;

  const int base = 6 + idx * 3;
  P.block(base, 0, 3, P.cols()).setZero();
  P.block(0, base, P.rows(), 3).setZero();
  P.block(base, base, 3, 3) = initial_variance * Eigen::Matrix3d::Identity();

  return idx;
}

void State::remove_landmark_by_id(size_t id) {
  for (int i = 0; i < N_MAX; ++i) {
    if (is_active[i] && feature_ids[i] == id) {
      is_active[i] = false;
      feature_ids[i] = 0;
      track_counts[i] = 0;
      p_L[i].setZero();

      const int base = 6 + i * 3;
      P.block(base, 0, 3, P.cols()).setZero();
      P.block(0, base, P.rows(), 3).setZero();
      P.block(base, base, 3, 3) = 1e-8 * Eigen::Matrix3d::Identity();
      return;
    }
  }
}

int State::get_slot_by_id(size_t id) const {
  for (int i = 0; i < N_MAX; ++i) {
    if (is_active[i] && feature_ids[i] == id) {
      return i;
    }
  }
  return -1;
}

int State::get_active_landmark_count() const {
  int cnt = 0;
  for (int i = 0; i < N_MAX; ++i) {
    if (is_active[i]) {
      ++cnt;
    }
  }
  return cnt;
}

}  // namespace hno_slam
