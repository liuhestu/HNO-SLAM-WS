#pragma once

#include <cstddef>

#include <Eigen/Core>

namespace hno_slam {

class State {
 public:
  // Stereo observation packet used by upper modules.
  struct StereoObservation {
    int track_id = -1;
    double u_l = 0.0;
    double v_l = 0.0;
    double u_r = 0.0;
    double v_r = 0.0;
  };

  // Maximum number of simultaneously active landmarks.
  static constexpr int N_MAX = 50;

  // Nonlinear Lie-group part.
  Eigen::Matrix3d R_hat;

  // LTV part managed by covariance P.
  Eigen::Vector3d p_hat;
  Eigen::Vector3d v_hat;
  Eigen::Vector3d g_hat;

  // Landmark fixed-size pool.
  Eigen::Vector3d p_L[N_MAX];
  bool is_active[N_MAX];
  size_t feature_ids[N_MAX];
  int track_counts[N_MAX];

  // Full fixed-size covariance container.
  Eigen::MatrixXd P;

  State();

  int add_landmark(size_t id, const Eigen::Vector3d& p_w_init, double initial_variance);
  void remove_landmark_by_id(size_t id);
  int get_slot_by_id(size_t id) const;
  int get_active_landmark_count() const;
};

}  // namespace hno_slam
