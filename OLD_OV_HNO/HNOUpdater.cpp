/*序贯更新 + 卡方检验*/

#include "HNOUpdater.h"
#include <iostream>
#include <iomanip>
#include <vector>

using namespace ov_hno;
using namespace Eigen;

HNOUpdater::HNOUpdater() {
    // 默认外参为单位阵 (需通过 setExtrinsics 设置)
    R_C2B_left.setIdentity(); pc_left.setZero();
    R_C2B_right.setIdentity(); pc_right.setZero();
}

void HNOUpdater::setExtrinsics(const std::map<size_t, Eigen::Matrix4d>& T_C2B_map) {
    if(T_C2B_map.count(0)) {
        R_C2B_left = T_C2B_map.at(0).block<3,3>(0,0);
        pc_left = T_C2B_map.at(0).block<3,1>(0,3);
    }
    if(T_C2B_map.count(1)) {
        R_C2B_right = T_C2B_map.at(1).block<3,3>(0,0);
        pc_right = T_C2B_map.at(1).block<3,1>(0,3);
    }
    has_stereo_extrinsics = (T_C2B_map.count(0) && T_C2B_map.count(1));
}

Eigen::Matrix3d HNOUpdater::project_pi(const Eigen::Vector3d& x) {
    if(x.norm() < 1e-5) return Eigen::Matrix3d::Identity(); // 增加数值保护
    Eigen::Vector3d n = x.normalized();     //确保x是单位向量
    return Eigen::Matrix3d::Identity() - n * n.transpose();
}

void HNOUpdater::update(std::shared_ptr<HNOState> state,
                        const std::vector<HNOObservation>& observations) {
    static int update_counter = 0; // Warm-up counter
    static int print_counter = 0;

    int N = observations.size();
    if(N == 0) return;

    // 归一化平面像素噪声标准差 Cov(ny)
    double sigma_pix = 2.0 / focal_length; 
    double sigma_pix_sq = sigma_pix * sigma_pix;   

    // 卡方检验阈值 (3自由度, 99%置信度)
    const double chi2_threshold = 25; 
    const double chi2_gate = 15.0; // 实际工作门限

    // 统计有效更新点数
    int update_valid = 0;
    int reject_chi2 = 0, reject_nan = 0, reject_trunc_p = 0, reject_trunc_r = 0;
    double chi2_max = 0.0, chi2_max_rej = 0.0;
    double delta_p_max = 0.0, delta_r_max = 0.0;

    // --- 采用序贯更新 (Sequential Update) ---
    for (int i = 0; i < N; ++i) {
        const auto& feature = observations[i];
    
        // 1. 获取当前最新状态
        Eigen::Matrix3d R_hat_B2I = state->R_hat_B2I; 
        Eigen::Vector3d p_hat = state->p_hat;

        // 2. 准备路标 (Inertial Frame)
        // 使用当前的 e_hat 重构路标位置 p_i_hat = sum(p_ij * e_j_hat)
        double p_i1 = feature.xyz(0);
        double p_i2 = feature.xyz(1);
        double p_i3 = feature.xyz(2);

        Eigen::Vector3d pf_hat_I = p_i1 * state->e_hat[0] + 
                                   p_i2 * state->e_hat[1] + 
                                   p_i3 * state->e_hat[2];
        // 路标在机体系的估计位置 pf_hat_B = R_hat^T * (p_i_hat - p_hat)
        Eigen::Vector3d pf_hat_B = R_hat_B2I.transpose() * (pf_hat_I - p_hat);


        // 3. 计算残差
        // 左目残差，将相机系归一化坐标投影到机体系 pi(R_c*y_i)
        Eigen::Matrix3d pi_left = R_C2B_left * project_pi(feature.uv_left) * R_C2B_left.transpose();
        Eigen::Vector3d sigma_y_left = pi_left * (pf_hat_B - pc_left);

        // 右目残差，因为右目不一定有，所以必须赋初值
        Eigen::Matrix3d pi_right = Eigen::Matrix3d::Zero();
        Eigen::Vector3d sigma_y_right = Eigen::Vector3d::Zero();
        if (feature.isValidRight && has_stereo_extrinsics) {
            pi_right = R_C2B_right * project_pi(feature.uv_right) * R_C2B_right.transpose();
            sigma_y_right = pi_right * (pf_hat_B - pc_right);
        }
        // 总残差
        Eigen::Vector3d sigma_y_i = sigma_y_left + sigma_y_right;
        
        // 总投影算子 公式(27b)用到的
        Eigen::Matrix3d Pi_total = pi_left + pi_right;


        // 4. 计算观测噪声协方差 Q_i (3x3) 实际是Q^-1
        // 公式(27b) Q^-1 = Mt * Cov(ny) * Mt^T
        double dist_sq = pf_hat_B.squaredNorm(); 
        if(dist_sq < 0.1 || std::isnan(dist_sq)) dist_sq = 0.1;

        Eigen::Matrix3d Q_i = (dist_sq * sigma_pix_sq) * Pi_total;

        // Ensure Q_i is positive definite enough. 
        Q_i += 1e-8 * Eigen::Matrix3d::Identity();


        // 5. 构造雅可比 C_i (3x15)
        // 公式(12): C_i = [ Pi_total, -p_i1*Pi_total, -p_i2*Pi_total, -p_i3*Pi_total, 0 ]
        Eigen::Matrix<double, 3, 15> C_i; C_i.setZero();
        C_i.block<3,3>(0, 0) = Pi_total;          // Pos
        C_i.block<3,3>(0, 3) = -p_i1 * Pi_total; // e1
        C_i.block<3,3>(0, 6) = -p_i2 * Pi_total; // e2
        C_i.block<3,3>(0, 9) = -p_i3 * Pi_total; // e3
        

        // 6. 卡方检验并更新增益 K=[Kp, K1, K2, K3, Kv]
        // S_i = C_i * P * C_i^T + Q_i
        Eigen::Matrix<double, 15, 3> PHT = state->P * C_i.transpose();
        Eigen::Matrix3d S_i = C_i * PHT + Q_i;
        
        
        // 检查 NaN (第一道防线)
        if(S_i.hasNaN()) { reject_nan++; continue; }

        // S 求逆 (3x3 矩阵，LDLT 极快且稳定)
        Eigen::LLT<Eigen::Matrix3d> llt(S_i);
        if (llt.info() == Eigen::NumericalIssue) { reject_nan++; continue; } // 矩阵奇异，跳过

        // 计算马氏距离 (Chi-Square)
           double chi2 = sigma_y_i.transpose() * llt.solve(sigma_y_i);
           if(chi2 > chi2_max) chi2_max = chi2;

           if (std::isnan(chi2) || std::isinf(chi2) || chi2 > chi2_gate) {
               reject_chi2++;
               if(chi2 > chi2_max_rej) chi2_max_rej = chi2;
               continue;
           }

        // --- 通过检验，执行更新 ---
        update_valid++;

        // 公式(24) K = P * C^T * (C * P * C^T + Q)^-1
        // K = P H^T S^-1
        Eigen::Matrix<double, 15, 3> K = PHT * llt.solve(Eigen::Matrix3d::Identity());
        
        // 检查 K (第三道防线)
        if(K.hasNaN()) continue;


        // 7. 更新状态
        // 计算状态修正量 K*sigma_y (15x1 Body Frame Error)
        Eigen::VectorXd delta = K * sigma_y_i;
        
        // [CRITICAL FIX: Third Defense Line - Update Truncation]
        // Truncate explosive updates.
        // During stable flight (20Hz), corrections > 0.15m are likely errors.
           double delta_p = delta.segment<3>(0).norm();
           double delta_r = delta.segment<3>(3).norm();
           if(delta_p > delta_p_max) delta_p_max = delta_p;
           if(delta_r > delta_r_max) delta_r_max = delta_r;

           if (delta_p > 0.2) { 
               reject_trunc_p++;
               continue;
           }
           if (delta_r > 0.15) { // ~8 degree
               reject_trunc_r++;
               continue;
           }

        // p, e, v 都是定义在 Inertial Frame 的,delta 是定义在 Body Frame 的误差, dx_I = R * dx_B
        state->p_hat    += R_hat_B2I * delta.segment<3>(0);
        state->e_hat[0] += R_hat_B2I * delta.segment<3>(3);
        state->e_hat[1] += R_hat_B2I * delta.segment<3>(6);
        state->e_hat[2] += R_hat_B2I * delta.segment<3>(9);
        state->v_hat    += R_hat_B2I * delta.segment<3>(12);

        // 8. 更新协方差
        // 公式(25) P = (I - K*C) * P * (I-KC)' + KQK'
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * C_i;
        state->P = I_KH * state->P * I_KH.transpose() + K * Q_i * K.transpose(); 

        // 强制对称, 防止长期运行积累不对称误差
        state->P = 0.5 * (state->P + state->P.transpose());
        
        // 协方差防爆 (第五道防线)
        if (state->P.diagonal().minCoeff() < 0) {
             // 极罕见情况：重置 P
             state->P.setIdentity(); state->P *= 1e-4;
        }
    }

    // 防止P过自信 (Covariance Limiting)，保持对新观测的敏感度
    // 1e-9 对应标准差 0.03mm 或 0.0017度，是一个安全的数值下限
    for(int i=0; i<15; i++) {
        if(state->P(i,i) < 1e-9) {
            state->P(i,i) = 1e-9;
        }
    }

    if (update_valid > 0) update_counter++;
    print_counter++;
    
    // 每帧更新结束后，强制约束 e_hat 必须也是单位正交基
    // state->enforce_structure();

    // 节流日志：每 30 次尝试更新打印一次统计
    if(print_counter % 30 == 0) {
        std::cout << std::fixed << std::setprecision(3)
                  << "[HNOUpdater] obs " << N
                  << " accepted " << update_valid
                  << " chi2_max " << chi2_max
                  << " chi2_rej " << reject_chi2 << " max " << chi2_max_rej
                  << " trunc_p " << reject_trunc_p
                  << " trunc_r " << reject_trunc_r
                  << " nan " << reject_nan
                  << " dP_max " << delta_p_max
                  << " dR_max " << delta_r_max
                  << " P_pos_diag " << state->P(0,0) << "," << state->P(1,1) << "," << state->P(2,2)
                  << std::endl;
    }
}
