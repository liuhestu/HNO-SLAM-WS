#include "System.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <opencv2/core.hpp>

#include "../thirdparty/cam/CamEqui.h"
#include "../thirdparty/cam/CamRadtan.h"

namespace hno_slam {

namespace {

double read_double_or(const cv::FileNode& node, const std::string& key, double def) {
  if (!node.empty() && !node[key].empty()) {
    return static_cast<double>(node[key]);
  }
  return def;
}

std::string read_string_or(const cv::FileNode& node, const std::string& key, const std::string& def) {
  if (!node.empty() && !node[key].empty()) {
    return static_cast<std::string>(node[key]);
  }
  return def;
}

Eigen::Matrix4d read_mat4(const cv::FileNode& node, const std::string& key, const Eigen::Matrix4d& def) {
  if (node.empty() || node[key].empty()) {
    return def;
  }
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out(r, c) = static_cast<double>(node[key][r][c]);
    }
  }
  return out;
}

Eigen::Matrix3d read_mat3(const cv::FileNode& node, const std::string& key, const Eigen::Matrix3d& def) {
  if (node.empty() || node[key].empty()) {
    return def;
  }
  Eigen::Matrix3d out = Eigen::Matrix3d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out(r, c) = static_cast<double>(node[key][r][c]);
    }
  }
  return out;
}

}  // namespace

System::System(const std::string& config_path)
    : is_initialized_(false),
      last_img_time_(-1.0),
      bg_gyro_(Eigen::Vector3d::Zero()),
  ba_accel_(Eigen::Vector3d::Zero()),
  Tw_(Eigen::Matrix3d::Identity()),
  Ta_(Eigen::Matrix3d::Identity()),
  R_imu_to_gyro_(Eigen::Matrix3d::Identity()),
  R_imu_to_acc_(Eigen::Matrix3d::Identity()),
  Tg_(Eigen::Matrix3d::Zero()),
  T_I_B_(Eigen::Matrix4d::Identity()),
  R_B_I_(Eigen::Matrix3d::Identity()),
  has_track_viz_(false) {
  std::vector<std::shared_ptr<ov_core::CamBase>> cameras;
  std::vector<Eigen::Matrix4d> T_C_B;

  double k_R = 1.0;
  double noise_acc = 2.0e-3;
  double noise_gyro = 1.6968e-4;
  double sigma_pix = 1.0;

  cv::FileStorage fs_est(config_path, cv::FileStorage::READ);
  if (fs_est.isOpened()) {
    k_R = read_double_or(fs_est.root(), "k_R", k_R);
    sigma_pix = read_double_or(fs_est.root(), "up_slam_sigma_px", sigma_pix);

    const std::string folder = config_path.substr(0, config_path.find_last_of('/')) + "/";
    const std::string imucam_path = folder + read_string_or(fs_est.root(), "relative_config_imucam", "kalibr_imucam_chain.yaml");
    const std::string imu_path = folder + read_string_or(fs_est.root(), "relative_config_imu", "kalibr_imu_chain.yaml");

    cv::FileStorage fs_imu(imu_path, cv::FileStorage::READ);
    if (fs_imu.isOpened() && !fs_imu["imu0"].empty()) {
      const cv::FileNode imu0 = fs_imu["imu0"];
      noise_acc = read_double_or(imu0, "accelerometer_noise_density", noise_acc);
      noise_gyro = read_double_or(imu0, "gyroscope_noise_density", noise_gyro);
      Tw_ = read_mat3(imu0, "Tw", Eigen::Matrix3d::Identity());
      Ta_ = read_mat3(imu0, "Ta", Eigen::Matrix3d::Identity());
      R_imu_to_gyro_ = read_mat3(imu0, "R_IMUtoGYRO", Eigen::Matrix3d::Identity());
      R_imu_to_acc_ = read_mat3(imu0, "R_IMUtoACC", Eigen::Matrix3d::Identity());
      Tg_ = read_mat3(imu0, "Tg", Eigen::Matrix3d::Zero());
      T_I_B_ = read_mat4(imu0, "T_i_b", Eigen::Matrix4d::Identity());
      R_B_I_ = T_I_B_.block<3, 3>(0, 0).transpose();
    }

    cv::FileStorage fs_cam(imucam_path, cv::FileStorage::READ);
    if (fs_cam.isOpened()) {
      for (int cam_idx = 0; cam_idx < 2; ++cam_idx) {
        const std::string cam_name = "cam" + std::to_string(cam_idx);
        const cv::FileNode cam_node = fs_cam[cam_name];
        if (cam_node.empty()) {
          continue;
        }

        std::vector<int> res(2, 0);
        std::vector<double> intr(4, 0.0);
        std::vector<double> dist(4, 0.0);
        std::string model = "radtan";
        cam_node["resolution"] >> res;
        cam_node["intrinsics"] >> intr;
        cam_node["distortion_coeffs"] >> dist;
        cam_node["distortion_model"] >> model;

        std::shared_ptr<ov_core::CamBase> cam;
        if (model == "equidistant") {
          cam = std::make_shared<ov_core::CamEqui>(res[0], res[1]);
        } else {
          cam = std::make_shared<ov_core::CamRadtan>(res[0], res[1]);
        }

        Eigen::MatrixXd calib(8, 1);
        calib << intr[0], intr[1], intr[2], intr[3], dist[0], dist[1], dist[2], dist[3];
        cam->set_value(calib);
        cameras.push_back(cam);

        const Eigen::Matrix4d T_I_C = read_mat4(cam_node, "T_imu_cam", Eigen::Matrix4d::Identity());
        const Eigen::Matrix4d T_C_I = T_I_C.inverse();
        const Eigen::Matrix4d T_C_B_i = T_C_I * T_I_B_;
        T_C_B.push_back(T_C_B_i);
      }
    }
  }

  if (cameras.size() < 2) {
    cameras.clear();
    T_C_B.clear();
    for (int i = 0; i < 2; ++i) {
      auto cam = std::make_shared<ov_core::CamRadtan>(752, 480);
      Eigen::MatrixXd calib(8, 1);
      calib << 458.0, 458.0, 376.0, 240.0, 0.0, 0.0, 0.0, 0.0;
      cam->set_value(calib);
      cameras.push_back(cam);
      T_C_B.push_back(Eigen::Matrix4d::Identity());
    }
  }

  state_ = std::make_shared<State>();
  initializer_ = std::make_shared<Initializer>();
  frontend_ = std::make_shared<Frontend>(cameras, T_C_B);
  observer_ = std::make_shared<Observer>(k_R, noise_acc, noise_gyro, sigma_pix, T_C_B);
}

void System::feed_imu(const ov_core::ImuData& msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  imu_buf_.push_back(msg);
  initializer_->feed_imu(msg);
}

ov_core::ImuData System::interpolate_imu(const ov_core::ImuData& a,
                                         const ov_core::ImuData& b,
                                         double t) const {
  ov_core::ImuData out;
  out.timestamp = t;
  const double dt = b.timestamp - a.timestamp;
  if (dt <= 1e-12) {
    out.wm = a.wm;
    out.am = a.am;
    return out;
  }
  const double alpha = (t - a.timestamp) / dt;
  out.wm = (1.0 - alpha) * a.wm + alpha * b.wm;
  out.am = (1.0 - alpha) * a.am + alpha * b.am;
  return out;
}

void System::calibrate_imu(const ov_core::ImuData& raw,
                           const Eigen::Vector3d& bg,
                           const Eigen::Vector3d& ba,
                           Eigen::Vector3d& omega_b,
                           Eigen::Vector3d& acc_b) const {
  const Eigen::Vector3d omega_i = R_imu_to_gyro_ * (Tw_ * (raw.wm - bg));
  const Eigen::Vector3d acc_i = R_imu_to_acc_ * (Ta_ * (raw.am - ba)) + Tg_ * omega_i;
  omega_b = R_B_I_ * omega_i;
  acc_b = R_B_I_ * acc_i;
}

void System::feed_camera(const ov_core::CameraData& msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!is_initialized_) {
    frontend_->process_frame(msg);
    std::unordered_set<size_t> active_ids_for_viz;
    for (int i = 0; i < State::N_MAX; ++i) {
      if (state_->is_active[i] && state_->track_counts[i] >= 3) {
        active_ids_for_viz.insert(state_->feature_ids[i]);
      }
    }
    has_track_viz_ = frontend_->render_track_overlay(last_track_viz_, active_ids_for_viz);

    double init_time = -1.0;
    if (initializer_->initialize(*state_, bg_gyro_, ba_accel_, init_time)) {
      is_initialized_ = true;
      last_img_time_ = msg.timestamp;
      while (!imu_buf_.empty() && imu_buf_.front().timestamp <= msg.timestamp) {
        imu_buf_.pop_front();
      }
    }
    return;
  }

  if (last_img_time_ < 0.0) {
    last_img_time_ = msg.timestamp;
  }

  const double t0 = last_img_time_;
  const double t1 = msg.timestamp;
  if (t1 > t0 && !imu_buf_.empty()) {
    int idx_after_t0 = -1;
    for (size_t i = 0; i < imu_buf_.size(); ++i) {
      if (imu_buf_[i].timestamp > t0) {
        idx_after_t0 = static_cast<int>(i);
        break;
      }
    }

    ov_core::ImuData imu_prev;
    if (idx_after_t0 <= 0) {
      imu_prev = imu_buf_.front();
      imu_prev.timestamp = t0;
    } else {
      imu_prev = interpolate_imu(imu_buf_[idx_after_t0 - 1], imu_buf_[idx_after_t0], t0);
    }

    double t_cur = t0;
    int i = std::max(0, idx_after_t0);
    for (; i < static_cast<int>(imu_buf_.size()) && imu_buf_[i].timestamp < t1; ++i) {
      const ov_core::ImuData& imu_i = imu_buf_[i];
      if (imu_i.timestamp <= t_cur) {
        imu_prev = imu_i;
        continue;
      }

      const double dt = imu_i.timestamp - t_cur;
      Eigen::Vector3d omega;
      Eigen::Vector3d accel;
      calibrate_imu(imu_prev, bg_gyro_, ba_accel_, omega, accel);
      observer_->propagate(*state_, omega, accel, dt);

      t_cur = imu_i.timestamp;
      imu_prev = imu_i;
    }

    if (t_cur < t1) {
      ov_core::ImuData imu_end = imu_prev;
      if (i >= 1 && i < static_cast<int>(imu_buf_.size())) {
        imu_end = interpolate_imu(imu_buf_[i - 1], imu_buf_[i], t1);
      }

      const double dt = t1 - t_cur;
      Eigen::Vector3d omega;
      Eigen::Vector3d accel;
      calibrate_imu(imu_end, bg_gyro_, ba_accel_, omega, accel);
      observer_->propagate(*state_, omega, accel, dt);
    }
  }

  std::vector<FeatureObs> obs = frontend_->process_frame(msg);

  std::unordered_set<size_t> tracked_ids;
  tracked_ids.reserve(obs.size());
  for (const auto& ob : obs) {
    tracked_ids.insert(ob.id);
  }

  std::vector<size_t> ids_to_remove;
  for (int i = 0; i < State::N_MAX; ++i) {
    if (!state_->is_active[i]) {
      continue;
    }
    const size_t id = state_->feature_ids[i];
    if (tracked_ids.find(id) == tracked_ids.end()) {
      ids_to_remove.push_back(id);
    }
  }
  for (size_t id : ids_to_remove) {
    state_->remove_landmark_by_id(id);
  }

  if (!obs.empty()) {
    observer_->update(*state_, obs);
  }

  std::unordered_set<size_t> active_ids;
  for (int i = 0; i < State::N_MAX; ++i) {
    if (state_->is_active[i] && state_->track_counts[i] >= 3) {
      active_ids.insert(state_->feature_ids[i]);
    }
  }
  has_track_viz_ = frontend_->render_track_overlay(last_track_viz_, active_ids);

  while (!imu_buf_.empty() && imu_buf_.front().timestamp < msg.timestamp) {
    imu_buf_.pop_front();
  }

  last_img_time_ = msg.timestamp;
}

std::shared_ptr<State> System::get_state() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return state_;
}

bool System::get_track_visualization(cv::Mat& img) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!has_track_viz_ || last_track_viz_.empty()) {
    return false;
  }
  img = last_track_viz_.clone();
  return true;
}

}  // namespace hno_slam
