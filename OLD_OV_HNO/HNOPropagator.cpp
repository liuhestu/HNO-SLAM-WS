#include "HNOPropagator.h"

using namespace ov_hno;

HNOPropagator::HNOPropagator() {
    // 权重初始化
    rho << 0.5, 0.3, 0.2;

    // 初始化 Cov_nx 为一个小量，
    // 防止忘记调用 setNoiseParams 时出错
    Cov_nx.setIdentity();
    Cov_nx *= 1e-4;
}

void HNOPropagator::setNoiseParams(const NoiseParams& params) {
    double var_acc = params.noise_acc * params.noise_acc;
    double var_gyro = params.noise_gyro * params.noise_gyro;

    Cov_nx.setZero();
    Cov_nx.block<3,3>(0, 0) =  35*var_gyro * Eigen::Matrix3d::Identity();
    Cov_nx.block<3,3>(3, 3) =  35*var_acc  * Eigen::Matrix3d::Identity();

    // 打印调试信息
    std::cout << "HNO Noise Params Set: Var_Acc=" << var_acc << " Var_Gyr=" << var_gyro << std::endl;
}

Eigen::Matrix3d HNOPropagator::skew(const Eigen::Vector3d& v_hat) {
    Eigen::Matrix3d S;
    S << 0, -v_hat(2), v_hat(1), 
        v_hat(2), 0, -v_hat(0), 
        -v_hat(1), v_hat(0), 0;
    return S;
}

void HNOPropagator::propagate(std::shared_ptr<HNOState> state, 
                              const Eigen::Vector3d& omega_m, 
                              const Eigen::Vector3d& acc_m, 
                              double dt) {
    
    // state.h中定义的e放在private域，这里需要重新定义
    Eigen::Vector3d e[3];
    e[0] << 1,0,0; e[1] << 0,1,0; e[2] << 0,0,1;

    // 局部临时变量，精简变量长度
    const Eigen::Matrix3d R_hat_B2I = state->R_hat_B2I;
    const Eigen::Vector3d p_hat     = state->p_hat;
    const Eigen::Vector3d v_hat     = state->v_hat;
    const Eigen::Vector3d e_hat[3] = {state->e_hat[0], state->e_hat[1], state->e_hat[2]};

    // 1. 减去零偏的惯性测量
    Eigen::Vector3d omega = omega_m - state->bg;
    Eigen::Vector3d accel = acc_m - state->ba;

    // 2. 公式(7) sigma_R = k_R/2 * Sum(rho[i] * e_hat[i] x e[i])
    Eigen::Vector3d sigma_R = Eigen::Vector3d::Zero();
    for(int i=0; i<3; i++) {
        sigma_R += rho[i] * e_hat[i].cross(e[i]);
    }
    sigma_R *= (0.5 * k_R);



// =======================================================
    // // [新增] 在线零偏估计 (Heuristic Bias Update)
    // // 类似于 Mahony 滤波器的积分项：零偏是误差的积分
    // // k_bias 不需要很大，只需要能跟上漂移的速度即可 (例如 0.01 ~ 0.1)
    
    // double k_bias_g = 0.1; // 陀螺仪零偏增益
    // // double k_bias_a = 0.01; // 加速度计零偏增益 (通常很难估准，建议先只估陀螺仪)

    // // 将惯性系的误差 sigma_R 投影回机体系
    // Eigen::Vector3d sigma_R_body = R_hat_B2I.transpose() * sigma_R;
    
    // // 更新陀螺仪零偏 (负反馈)
    // // 注意方向：sigma_R 是修正量。如果我们需要正向修正 R，说明之前的 w 算小了？
    // // 这里的符号通常需要调试，但在 HNO 架构下，sigma_R_body 被加到了 w 上。
    // // 如果 bias 导致 w 偏大，我们需要把 bias 调大。
    // state->bg += -k_bias_g * sigma_R_body * dt;
    
    // // 限制零偏范围 (防止发散)
    // if (state->bg.norm() > 0.5) state->bg.setZero(); // 保护措施
    // // =======================================================





    // 3. 连续动力学积分 (公式 6)
    Eigen::Matrix3d Sigma_R = skew(sigma_R);

    // 6(b) p_hat_dot = v_hat + Sigma_R * p_hat
    Eigen::Vector3d dp = v_hat + Sigma_R * p_hat;

    // g_hat = Sum( gravity[i] * e_hat[i] )
    Eigen::Vector3d g_hat = Eigen::Vector3d::Zero();
    for(int i=0; i<3; i++) g_hat += gravity[i] * e_hat[i];

    // 6(c) v_hat_dot = g_hat + R_hat_B2I * accel + Sigma_R * v_hat
    Eigen::Vector3d dv = g_hat + R_hat_B2I * accel + Sigma_R * v_hat;

    // 6(d) de_hat[i] = Sigma_R * e_hat[i]
    Eigen::Vector3d de_hat[3];
    for(int i=0; i<3; i++) de_hat[i] = Sigma_R * e_hat[i];
    
    // 4. 更新状态p, v, e1, e2, e3
    // 对于高频数据，欧拉积分通常是足够的
    state->p_hat += dp * dt;
    state->v_hat += dv * dt;
    for(int i=0; i<3; i++) state->e_hat[i] += de_hat[i] * dt;


    // 5. 姿态更新
    // 公式6(a) R_hat_B2I_dot = R_hat_B2I * skew(omega + R_hat_B2I^T * sigma_R)
    // 直接对R线性积分破坏李群正交性
    Eigen::Vector3d omega_total = omega + R_hat_B2I.transpose() * sigma_R;
    Eigen::Vector3d angle_axis = omega_total * dt;
    double theta = angle_axis.norm();
    // 罗德里格斯公式，或指数映射(慢)。转得极慢就不更新
    if(theta > 1e-8) {
        Eigen::Vector3d k = angle_axis / theta;
        // A 使用 Eigen 库函数更新 R
        state->R_hat_B2I = state->R_hat_B2I * Eigen::AngleAxisd(theta, k).toRotationMatrix();

        // // B 手写公式
        // // Eigen::Vector3d k = angle_axis / theta;
        // Eigen::Matrix3d K = skew(k);
        // Eigen::Matrix3d R_inc = Eigen::Matrix3d::Identity() + std::sin(theta)*K + (1.0 - std::cos(theta)) * K * K;
        // // 更新 R (右乘，因为角速度是在机体系定义的)
        // state->R_hat_B2I = state->R_hat_B2I * R_inc;
    }

    // 结构强制约束：归一化 e_hat 防止漂移
    // state->enforce_structure();


    // 6. 协方差传播 计算A(t), 公式(10)
    Eigen::Matrix<double, 15, 15> A; 
    A.setZero();
    // Diagonal Blocks: -omega× (Body Error dynamics)
    for(int k=0; k<5; k++) A.block<3,3>(3*k, 3*k) = -skew(omega);

    A.block<3,3>(0, 12) = Eigen::Matrix3d::Identity(); 
    A.block<3,3>(12, 3) = gravity(0) * Eigen::Matrix3d::Identity();
    A.block<3,3>(12, 6) = gravity(1) * Eigen::Matrix3d::Identity();
    A.block<3,3>(12, 9) = gravity(2) * Eigen::Matrix3d::Identity();


    // 7. 计算时变过程噪声矩阵 V(t)
    // 公式(27a) Vt = Gt * Cov(nx) * Gt^T

    // R,p,v,e刚才更新过了，重新获取最新值
    Eigen::Matrix3d Rt = state->R_hat_B2I.transpose();
    Eigen::Vector3d p  = state->p_hat;
    Eigen::Vector3d v  = state->v_hat;
    Eigen::Vector3d e1 = state->e_hat[0];
    Eigen::Vector3d e2 = state->e_hat[1];
    Eigen::Vector3d e3 = state->e_hat[2];

    // 构造 Gt 矩阵 (15x6)
    Eigen::Matrix<double, 15, 6> Gt;
    Gt.setZero();

    // 填充左列 (Gyro Noise 部分)
    Gt.block<3,3>(0, 0) = skew(Rt * p);
    Gt.block<3,3>(3, 0) = skew(Rt * e1);
    Gt.block<3,3>(6, 0) = skew(Rt * e2);
    Gt.block<3,3>(9, 0) = skew(Rt * e3);
    Gt.block<3,3>(12, 0) = skew(Rt * v);

    // 填充右列 (Accel Noise 部分)
    Gt.block<3,3>(12, 3) = -Eigen::Matrix3d::Identity();

    // V = G Q G^T 时负号会被抵消,这里就不Gt=-Gt了

    // 计算 V(t)
    // 维度验证: (15x6) * (6x6) * (6x15) = (15x15)
    Eigen::Matrix<double, 15, 15> Vt = Gt * Cov_nx * Gt.transpose();
    
    // 为了数值稳定性，可以在对角线加上极小的正数（Regularization）
    // Vt += 1e-12 * Eigen::Matrix<double, 15, 15>::Identity();


    // 8. 传播误差协方差P
    // 公式(9) Propagate Covariance P using Process Noise V
    state->P = RK4(A, state->P, Vt, dt);
    // state->P += (A * state->P + state->P * A.transpose() + V) * dt;
    state->P = 0.5 * (state->P + state->P.transpose());
    }

Eigen::Matrix<double, 15, 15> HNOPropagator::RK4(const Eigen::Matrix<double, 15, 15>& A, 
                                                 const Eigen::Matrix<double, 15, 15>& P, 
                                                 const Eigen::Matrix<double, 15, 15>& Vt, 
                                                 double dt) {
    // 定义导数函数 f(P) = A*P + P*A^T + V
    // 利用 P 的对称性，(AP)^T = P^T A^T = P A^T
    auto derivative = [&](const Eigen::Matrix<double, 15, 15>& P_curr) -> Eigen::Matrix<double, 15, 15> {
        Eigen::Matrix<double, 15, 15> tmp = A * P_curr;
        return tmp + tmp.transpose() + Vt;
    };

    Eigen::Matrix<double, 15, 15> k1 = derivative(P);
    Eigen::Matrix<double, 15, 15> k2 = derivative(P + 0.5 * dt * k1);
    Eigen::Matrix<double, 15, 15> k3 = derivative(P + 0.5 * dt * k2);
    Eigen::Matrix<double, 15, 15> k4 = derivative(P + dt * k3);

    return P + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}