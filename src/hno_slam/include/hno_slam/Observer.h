#pragma once

#include <vector>

#include <Eigen/Core>

#include "Frontend.h"
#include "State.h"

namespace hno_slam {

class Observer {
 public:
  Observer(double k_R,
           double noise_acc,
           double noise_gyro,
           double sigma_pix,
           const std::vector<Eigen::Matrix4d>& T_C_B);

  void propagate(State& state,
                 const Eigen::Vector3d& omega_m,
                 const Eigen::Vector3d& acc_m,
                 double dt);
  void update(State& state, const std::vector<FeatureObs>& observations);

 private:
  Eigen::Matrix3d skew(const Eigen::Vector3d& v) const;
  Eigen::Matrix3d project_pi(const Eigen::Vector3d& x) const;
  bool triangulate_stereo(const Eigen::Vector2d& uv_l_norm,
                          const Eigen::Vector2d& uv_r_norm,
                          Eigen::Vector3d& p_c_left) const;

  Eigen::Vector3d gravity_w_;
  double k_R_;
  Eigen::MatrixXd V_noise_;
  double sigma_pix_;
  std::vector<Eigen::Matrix4d> T_C_B_;
};

}  // namespace hno_slam
