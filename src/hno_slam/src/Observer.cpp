#include "Observer.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

namespace hno_slam {

// 观测器构造函数。
// 对应论文：系统噪声建模部分（IMU 加计/陀螺白噪声进入连续时间误差协方差传播）。
Observer::Observer(double k_R,
                   double noise_acc,
                   double noise_gyro,
                   double sigma_pix,
                   double sigma_p_proc,
                   const std::vector<Eigen::Matrix4d>& T_C_B)
    : gravity_w_(0.0, 0.0, -9.81),
      k_R_(k_R),
      sigma_pix_(sigma_pix),
      sigma_p_proc_(sigma_p_proc),
      T_C_B_(T_C_B) {
  // V_noise_ 是传播方程中的过程噪声协方差（仅作用在主状态 v/g 对应子块）。
  // 可调参数：noise_acc, noise_gyro（越大代表对 IMU 越不信任）。
  // 路标块的过程噪声 sigma_p_proc_ 在 propagate 中按活跃槽位注入，
  // 以满足论文 Theorem 1 对 V(t) ≻ 0 的一致正定性要求。
  V_noise_ = Eigen::MatrixXd::Zero(6, 6);
  V_noise_.block<3, 3>(0, 0) = (noise_acc * noise_acc) * Eigen::Matrix3d::Identity();
  V_noise_.block<3, 3>(3, 3) = (noise_gyro * noise_gyro) * Eigen::Matrix3d::Identity();
}

// 向量叉乘矩阵 [v]x。
// 对应论文：李代数/旋转动力学中的反对称矩阵记号。
Eigen::Matrix3d Observer::skew(const Eigen::Vector3d& v) const {
  Eigen::Matrix3d S;
  S << 0.0, -v(2), v(1),
      v(2), 0.0, -v(0),
      -v(1), v(0), 0.0;
  return S;
}

// 投影矩阵 Pi(x)=I-xx^T/||x||^2，将残差投影到射线正交平面。
// 对应论文：bearing-only 观测误差的投影算子（双目每目各一个 Pi）。
Eigen::Matrix3d Observer::project_pi(const Eigen::Vector3d& x) const {
  const double norm2 = x.squaredNorm();
  if (norm2 <= 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::Matrix3d::Identity() - (x * x.transpose()) / norm2;
}

// 双目三角化：由左右归一化像点恢复左相机坐标系下 3D 点。
// 对应论文：新特征初始化阶段（几何恢复，不属于滤波主更新方程）。
bool Observer::triangulate_stereo(const Eigen::Vector2d& uv_l_norm,
                                  const Eigen::Vector2d& uv_r_norm,
                                  Eigen::Vector3d& p_c_left) const {
  // 至少需要左右两目外参。
  if (T_C_B_.size() < 2) {
    return false;
  }

  const Eigen::Matrix3d R_cb_l = T_C_B_[0].block<3, 3>(0, 0);
  const Eigen::Vector3d p_cb_l = T_C_B_[0].block<3, 1>(0, 3);
  const Eigen::Matrix3d R_cb_r = T_C_B_[1].block<3, 3>(0, 0);
  const Eigen::Vector3d p_cb_r = T_C_B_[1].block<3, 1>(0, 3);

  const Eigen::Matrix3d R_rl = R_cb_r * R_cb_l.transpose();
  const Eigen::Vector3d p_rl = p_cb_r - R_rl * p_cb_l;

  const Eigen::Vector3d ray_l(uv_l_norm(0), uv_l_norm(1), 1.0);
  const Eigen::Vector3d ray_r(uv_r_norm(0), uv_r_norm(1), 1.0);

  // 最小二乘恢复双目尺度 lambda。
  // 按你的要求使用 SVD，相比 QR 在近退化基线/夹角下更稳健。
  Eigen::Matrix<double, 3, 2> A;
  A.col(0) = R_rl * ray_l;
  A.col(1) = -ray_r;
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::Vector2d lambda = svd.solve(-p_rl);

  // 双目前向可见性约束：两目深度尺度都必须为正。
  if (!(lambda(0) > 0.0 && lambda(1) > 0.0)) {
    return false;
  }

  p_c_left = lambda(0) * ray_l;
  // 深度门限（可调）：抑制过近/过远导致的数值不稳定。
  // 0.5m~15m 适配当前 EuRoC 场景，可按场景尺度调参。
  if (p_c_left(2) < 0.5 || p_c_left(2) > 15.0) {
    return false;
  }

  const Eigen::Vector3d p_c_right = R_rl * p_c_left + p_rl;
  // 右目也必须在成像平面前方。
  if (p_c_right(2) <= 1e-6) {
    return false;
  }

  // 右目重投影一致性检查（可调阈值）：过滤弱纹理误匹配。
  // 0.015 是归一化平面误差阈值，不是像素阈值。
  const Eigen::Vector2d uv_r_reproj(p_c_right(0) / p_c_right(2), p_c_right(1) / p_c_right(2));
  const double reproj_err = (uv_r_reproj - uv_r_norm).norm();
  return reproj_err < 0.015;
}

// 连续时间传播（离散积分实现）。
// 对应论文：观测器传播方程（R, p, v, g 以及路标 p_i 的连续动态）。
void Observer::propagate(State& state,
                         const Eigen::Vector3d& omega_m,
                         const Eigen::Vector3d& acc_m,
                         double dt) {
  // 非法时间步直接返回。
  if (dt <= 0.0) {
    return;
  }

  const Eigen::Matrix3d R_hat = state.R_hat;
  const Eigen::Vector3d p_hat = state.p_hat;
  const Eigen::Vector3d v_hat = state.v_hat;
  const Eigen::Vector3d g_hat = state.g_hat;

  // 姿态反馈项 sigma_R = k_R * (g_hat x g_ref)。
  // 对应论文：利用重力方向约束的姿态观测器注入项。
  // 关键可调参数：k_R（大则收敛快，但过大可能引入抖动）。
  const Eigen::Vector3d sigma_R = k_R_ * (g_hat.cross(gravity_w_));
  const Eigen::Matrix3d sigma_skew = skew(sigma_R);

  // 有效角速度：IMU 角速度 + 反馈项（旋到机体系）。
  const Eigen::Vector3d omega_eff = omega_m + R_hat.transpose() * sigma_R;
  const Eigen::Vector3d theta = omega_eff * dt;
  const double theta_norm = theta.norm();
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  if (theta_norm > 1e-12) {
    // 罗德里格斯公式：大角度更准确。
    const Eigen::Matrix3d K = skew(theta / theta_norm);
    dR = Eigen::Matrix3d::Identity() + std::sin(theta_norm) * K + (1.0 - std::cos(theta_norm)) * (K * K);
  } else {
    // 小角度一阶近似，避免数值除零。
    dR = Eigen::Matrix3d::Identity() + skew(theta);
  }

  state.R_hat = R_hat * dR;

  // 对应你要求的鲁棒性增强：每次传播后投影回 SO(3)。
  // 原因：长时间积分会引入非正交漂移，SVD 投影可强制 det(R)=+1。
  // 对应论文：旋转矩阵属于 SO(3) 的结构性约束。
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(state.R_hat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
  if (R_ortho.determinant() < 0.0) {
    R_ortho = svd.matrixU() * Eigen::Vector3d(1.0, 1.0, -1.0).asDiagonal() * svd.matrixV().transpose();
  }
  state.R_hat = R_ortho;

  // 主状态传播。
  // 对应论文：p,v,g 的连续动力学离散化。
  state.p_hat = p_hat + (sigma_skew * p_hat + v_hat) * dt;
  state.v_hat = v_hat + (sigma_skew * v_hat + g_hat + R_hat * acc_m) * dt;
  state.g_hat = g_hat + (sigma_skew * g_hat) * dt;

  // 地图点传播：采用与主状态一致的旋转反馈项。
  // 对应论文：p_i 动力学与误差系统统一写法。
  for (int i = 0; i < State::N_MAX; ++i) {
    if (!state.is_active[i]) {
      continue;
    }
    state.p_L[i] = state.p_L[i] + (sigma_skew * state.p_L[i]) * dt;
  }

  const int total_dim = 6 + 3 * State::N_MAX;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(total_dim, total_dim);
  const Eigen::Matrix3d omega_skew = skew(omega_m);
  // A 为误差线性化系统矩阵，对应论文 Eq. 35 / 37：
  //   A(t) = [[ D(t),      0_{6x3n}         ],
  //           [ B_n ⊗ I_3, I_n ⊗ (-[ω_B]×) ]]
  // 其中 D(t) = [[-[ω_B]×, I_3], [0, -[ω_B]×]]，B_n = [1_n, 0_{n×1}]。
  // 因此每个活跃路标 p_i 的误差动力学既含 -[ω_B]× 对角项，也有 I_3 耦合至速度 v。
  A.block<3, 3>(0, 0) = -omega_skew;           // v <- v
  A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();  // v <- g
  A.block<3, 3>(3, 3) = -omega_skew;           // g <- g

  for (int i = 0; i < State::N_MAX; ++i) {
    if (!state.is_active[i]) {
      continue;
    }
    const int idx = 6 + i * 3;
    A.block<3, 3>(idx, idx) = -omega_skew;              // p_i <- p_i
    A.block<3, 3>(idx, 0) = Eigen::Matrix3d::Identity();  // p_i <- v (B_n ⊗ I_3)
  }

  // 协方差传播：P_dot = A P + P A^T + V。
  // 对应论文 Theorem 1：V(t) 需一致正定，因此需要给活跃路标块注入
  // 过程噪声 sigma_p_proc_^2 · I_3，否则 3n 路标子块恒为 0 将违反正定性。
  Eigen::MatrixXd V = Eigen::MatrixXd::Zero(total_dim, total_dim);
  V.block(0, 0, 6, 6) = V_noise_;
  const double v_p = sigma_p_proc_ * sigma_p_proc_;
  for (int i = 0; i < State::N_MAX; ++i) {
    if (!state.is_active[i]) {
      continue;
    }
    const int idx = 6 + i * 3;
    V.block<3, 3>(idx, idx) = v_p * Eigen::Matrix3d::Identity();
  }
  state.P = state.P + (A * state.P + state.P * A.transpose() + V) * dt;

  // 数值对称化：抑制欧拉积分误差累积导致的非对称，保证 LDLT 分解稳定。
  state.P = 0.5 * (state.P + state.P.transpose());
}

// 视觉更新（信息形式，活跃子块解算）。
// 对应论文 Algorithm 1 的离散跳变步骤 12-20；并遵循 text.md 约束：
//   * 不更新 R_hat（旋转/平移解耦，见 Eq. 46）；
//   * 不更新 p_hat（K_p = 0，全局位置不可观）；
//   * 仅更新 v_hat、g_hat 以及活跃路标 p_i。
// 数值实现上，把矩阵压缩到 (6 + 3·n_active) 的活跃子块后再做信息形式分解，
// 避免对 156×156 的稀疏非活跃对角（1e-8 I）进行病态求逆。
void Observer::update(State& state, const std::vector<FeatureObs>& observations) {
  if (observations.empty()) {
    return;
  }

  // 更新阶段至少需要双目外参。
  if (T_C_B_.size() < 2) {
    return;
  }

  // 新地标初始协方差（可调）。
  // 大一些更保守，小一些更激进；会影响新点被视觉“拉动”的力度。
  constexpr double kInitialLandmarkVariance = 1.0;

  struct ValidObs {
    FeatureObs obs;
    int slot = -1;
  };
  std::vector<ValidObs> valid_obs;
  valid_obs.reserve(observations.size());

  // 1) 建立有效观测集：已有点直接使用；新点先三角化再入池。
  for (const auto& obs : observations) {
    int slot = state.get_slot_by_id(obs.id);

    bool added_new = false;
    if (slot < 0) {
      // 新点：必须先通过双目几何恢复 3D。
      Eigen::Vector3d p_c_left;
      if (!triangulate_stereo(obs.uv_l_norm, obs.uv_r_norm, p_c_left)) {
        continue;
      }

      // 左相机系 -> 机体系 -> 世界系，得到路标初值 p_w_init。
      const Eigen::Matrix3d R_cb = T_C_B_[0].block<3, 3>(0, 0);
      const Eigen::Vector3d p_cb = T_C_B_[0].block<3, 1>(0, 3);
      const Eigen::Matrix3d R_bc = R_cb.transpose();
      const Eigen::Vector3d p_bc = -R_cb.transpose() * p_cb;
      const Eigen::Vector3d p_w_init = state.R_hat * (R_bc * p_c_left + p_bc) + state.p_hat;

      // 入状态池（若池满则放弃该观测）。
      slot = state.add_landmark(obs.id, p_w_init, kInitialLandmarkVariance);
      if (slot < 0) {
        continue;
      }
      added_new = true;
    }

    if (slot < 0 || !state.is_active[slot]) {
      continue;
    }

    if (!added_new) {
      // 已有点累计跟踪次数，供系统做成熟度/淘汰策略。
      state.track_counts[slot] += 1;
    }
    valid_obs.push_back({obs, slot});
  }

  if (valid_obs.empty()) {
    return;
  }

  const Eigen::Matrix3d R_cb_l = T_C_B_[0].block<3, 3>(0, 0);
  const Eigen::Vector3d p_cb_l = T_C_B_[0].block<3, 1>(0, 3);
  const Eigen::Matrix3d R_cb_r = T_C_B_[1].block<3, 3>(0, 0);
  const Eigen::Vector3d p_cb_r = T_C_B_[1].block<3, 1>(0, 3);

  const Eigen::Matrix3d R_bc_l = R_cb_l.transpose();
  const Eigen::Matrix3d R_bc_r = R_cb_r.transpose();
  const Eigen::Vector3d p_bc_l = -R_cb_l.transpose() * p_cb_l;
  const Eigen::Vector3d p_bc_r = -R_cb_r.transpose() * p_cb_r;

  // 2) 活跃槽位索引化：把全局 slot 映射到紧凑的子块 local index。
  //    子块列布局：[ v (3) | g (3) | p_{s_0} (3) | p_{s_1} (3) | ... ]
  std::vector<int> used_slots;          // 保留本次观测涉及到的 slot（去重后按出现顺序）
  used_slots.reserve(valid_obs.size());
  std::vector<int> slot_to_local(State::N_MAX, -1);

  for (const auto& it : valid_obs) {
    const int slot = it.slot;
    if (slot < 0 || slot >= State::N_MAX || !state.is_active[slot]) {
      continue;
    }
    if (slot_to_local[slot] < 0) {
      slot_to_local[slot] = static_cast<int>(used_slots.size());
      used_slots.push_back(slot);
    }
  }

  const int n_active = static_cast<int>(used_slots.size());
  if (n_active == 0) {
    return;
  }

  const int sub_dim = 6 + 3 * n_active;

  // 3) 从全量 state.P 提取活跃子块 P_sub。
  //    列 [0..2] = v, [3..5] = g, [6+3k..8+3k] = 第 k 个活跃槽位。
  Eigen::MatrixXd P_sub = Eigen::MatrixXd::Zero(sub_dim, sub_dim);
  P_sub.block<6, 6>(0, 0) = state.P.block<6, 6>(0, 0);
  for (int k = 0; k < n_active; ++k) {
    const int g_k = 6 + used_slots[k] * 3;  // 全局行/列
    const int l_k = 6 + k * 3;              // 局部行/列
    P_sub.block<6, 3>(0, l_k) = state.P.block<6, 3>(0, g_k);
    P_sub.block<3, 6>(l_k, 0) = state.P.block<3, 6>(g_k, 0);
    for (int j = 0; j < n_active; ++j) {
      const int g_j = 6 + used_slots[j] * 3;
      const int l_j = 6 + j * 3;
      P_sub.block<3, 3>(l_k, l_j) = state.P.block<3, 3>(g_k, g_j);
    }
  }

  // 4) 构建观测方程 r = C_sub · δ + noise。每条观测贡献 3 维投影残差
  //    （左右目叠加），C 行仅在对应活跃路标列块放 Π_total（论文 Eq. 33）。
  const int M = static_cast<int>(valid_obs.size());
  Eigen::MatrixXd C_sub = Eigen::MatrixXd::Zero(3 * M, sub_dim);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(3 * M);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3 * M, 3 * M);

  int row_obs = 0;
  for (const auto& it : valid_obs) {
    const int slot = it.slot;
    if (slot < 0 || slot >= State::N_MAX || !state.is_active[slot]) {
      continue;
    }
    const int local = slot_to_local[slot];
    if (local < 0) {
      continue;
    }

    const Eigen::Vector3d p_i_hat = state.p_L[slot];
    const Eigen::Vector3d p_i_B = state.R_hat.transpose() * (p_i_hat - state.p_hat);

    const Eigen::Vector3d y_l(it.obs.uv_l_norm(0), it.obs.uv_l_norm(1), 1.0);
    const Eigen::Vector3d y_r(it.obs.uv_r_norm(0), it.obs.uv_r_norm(1), 1.0);

    // 机体系下的投影算子：Π_q = R_{bc_q} π(y_q) R_{bc_q}^T（论文 Eq. 21/27）。
    const Eigen::Matrix3d pi_l = R_bc_l * project_pi(y_l) * R_bc_l.transpose();
    const Eigen::Matrix3d pi_r = R_bc_r * project_pi(y_r) * R_bc_r.transpose();

    // 双目残差：σ_i^p = Σ_q Π_q · (R̂^T(p̂_i - p̂) - p_bc_q)。
    const Eigen::Vector3d residual_l = pi_l * (p_i_B - p_bc_l);
    const Eigen::Vector3d residual_r = pi_r * (p_i_B - p_bc_r);
    const Eigen::Vector3d r_i = residual_l + residual_r;
    const Eigen::Matrix3d Pi_total = pi_l + pi_r;

    const int row = row_obs * 3;
    r.segment<3>(row) = r_i;
    // 测量噪声：随深度平方缩放（远点噪声放大），并加 1e-8 防奇异。
    // 可调参数：sigma_pix_（前端归一化测量噪声标准差）。
    Q.block<3, 3>(row, row) = (p_i_B.squaredNorm() * sigma_pix_ * sigma_pix_) * Pi_total +
                              1e-8 * Eigen::Matrix3d::Identity();

    // C 在 [v, g] 列为 0，仅在该路标对应的 local 列块放 Π_total。
    C_sub.block<3, 3>(row, 6 + local * 3) = Pi_total;
    row_obs += 1;
  }

  if (row_obs == 0) {
    return;
  }

  if (row_obs != M) {
    C_sub.conservativeResize(3 * row_obs, Eigen::NoChange);
    r.conservativeResize(3 * row_obs);
    Q.conservativeResize(3 * row_obs, 3 * row_obs);
  }

  // 5) 信息形式解算：
  //    M = P_sub^{-1} + C_sub^T Q^{-1} C_sub,   δ = M^{-1} C_sub^T Q^{-1} r。
  const Eigen::MatrixXd I_sub = Eigen::MatrixXd::Identity(sub_dim, sub_dim);
  const Eigen::MatrixXd I_meas = Eigen::MatrixXd::Identity(Q.rows(), Q.cols());

  const Eigen::LDLT<Eigen::MatrixXd> ldltP(P_sub);
  const Eigen::MatrixXd P_inv = ldltP.solve(I_sub);

  const Eigen::LDLT<Eigen::MatrixXd> ldltQ(Q);
  const Eigen::MatrixXd Q_inv = ldltQ.solve(I_meas);

  Eigen::MatrixXd M_info = P_inv + C_sub.transpose() * Q_inv * C_sub;
  // 强制对称 + Tikhonov 正则化，保证 LDLT 分解稳定。
  M_info = 0.5 * (M_info + M_info.transpose());
  M_info.diagonal().array() += 1e-8;

  const Eigen::VectorXd rhs = C_sub.transpose() * Q_inv * r;
  const Eigen::LDLT<Eigen::MatrixXd> ldltM(M_info);
  const Eigen::VectorXd delta = ldltM.solve(rhs);

  const Eigen::Vector3d delta_v = delta.segment<3>(0);
  const Eigen::Vector3d delta_g = delta.segment<3>(3);

  // 6) 应用跳变增量（Algorithm 1 line 15-19，K_p = 0）。
  //    * v̂、ĝ 的 x-分量定义为 R̂^T(v - v̂)、R̂^T(g - ĝ)（真 - 估），
  //      故估计量用 `+= R̂ δ` 向真值靠拢。
  //    * p_i 的 x-分量为 R̂^T((p - p̂) - (p_i - p̂_i))，在 K_p = 0 下约等于
  //      R̂^T(p̂_i - p_i)（估 - 真），因此估计量需要 `-= R̂ δ_{p_i}`
  //      才能朝真值收敛。该负号与论文 Eq. 45 中 Γ 的显式负号对应。
  state.v_hat += state.R_hat * delta_v;
  state.g_hat += state.R_hat * delta_g;
  for (int k = 0; k < n_active; ++k) {
    const int slot = used_slots[k];
    const Eigen::Vector3d delta_pi = delta.segment<3>(6 + k * 3);
    state.p_L[slot] -= state.R_hat * delta_pi;
  }

  // 7) 后验协方差：P_sub^+ = M^{-1}，对称化后散射回 state.P 的活跃行/列。
  //    非活跃块保持不变（保留前一次传播得到的协方差，避免被污染）。
  Eigen::MatrixXd P_sub_post = ldltM.solve(I_sub);
  P_sub_post = 0.5 * (P_sub_post + P_sub_post.transpose());

  state.P.block<6, 6>(0, 0) = P_sub_post.block<6, 6>(0, 0);
  for (int k = 0; k < n_active; ++k) {
    const int g_k = 6 + used_slots[k] * 3;
    const int l_k = 6 + k * 3;
    state.P.block<6, 3>(0, g_k) = P_sub_post.block<6, 3>(0, l_k);
    state.P.block<3, 6>(g_k, 0) = P_sub_post.block<3, 6>(l_k, 0);
    for (int j = 0; j < n_active; ++j) {
      const int g_j = 6 + used_slots[j] * 3;
      const int l_j = 6 + j * 3;
      state.P.block<3, 3>(g_k, g_j) = P_sub_post.block<3, 3>(l_k, l_j);
    }
  }

  // 对角下界截断：避免数值噪声导致非正对角。
  for (int i = 0; i < state.P.rows(); ++i) {
    state.P(i, i) = std::max(state.P(i, i), 1e-9);
  }
}

}  // namespace hno_slam
