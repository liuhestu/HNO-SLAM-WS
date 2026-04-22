#ifndef HNO_STATE_H
#define HNO_STATE_H

#include <Eigen/Dense>
#include <iostream>

namespace ov_hno {

class HNOState {
public:
    // Eigen库要求的关键宏
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HNOState() {
        // 默认初始化，防止空指针，具体值会被 initialize 覆盖
        R_hat_B2I.setIdentity();
        p_hat.setZero();
        v_hat.setZero();

        e_hat[0] << 1, 0, 0;
        e_hat[1] << 0, 1, 0;
        e_hat[2] << 0, 0, 1;

        bg.setZero();
        ba.setZero();

        P.setIdentity();

          // 1. 位置误差: 初始虽然设为0，但我们允许它有 10cm 的不确定性
        P.block<3,3>(0,0) *= 1e-4; 
        
        // 2. 辅助向量 (e1,e2,e3) = 姿态误差:
        // 0.01 对应约 0.1弧度 (5.7度)
        P.block<3,3>(3,3) *= 0.01; 
        P.block<3,3>(6,6) *= 0.01; 
        P.block<3,3>(9,9) *= 0.01; 
        
        // 3. 速度误差: 初始可能不是绝对静止，给 0.1 m/s 的容错
        P.block<3,3>(12,12) *= 0.1;
    }

    // 变量定义
    // State Variables
    Eigen::Matrix3d R_hat_B2I;      // Body-to-Inertial Rotation
    Eigen::Vector3d p_hat;          // Inertial Position
    Eigen::Vector3d v_hat;          // Inertial Velocity
    Eigen::Vector3d e_hat[3];       // Auxiliary Vectors (INERTIAL FRAME ESTIMATE)

    // IMU Biases
    Eigen::Vector3d bg; // Gyro Bias
    Eigen::Vector3d ba; // Accel Bias

    // Covariance (15x15)
    // Order: [Pos(0-2), e1(3-5), e2(6-8), e3(9-11), Vel(12-14)]
    Eigen::Matrix<double, 15, 15> P;


    void initialize(const Eigen::Matrix3d& R0_B2I, const Eigen::Vector3d& p0_I, const Eigen::Vector3d& v0_I,
                    const Eigen::Vector3d& bg0, const Eigen::Vector3d& ba0) {

        R_hat_B2I = R0_B2I;
        p_hat = p0_I;
        v_hat = v0_I;
        bg = bg0;
        ba = ba0;

        // Reset e_hat to Identity Basis
        e_hat[0] << 1, 0, 0;
        e_hat[1] << 0, 1, 0;
        e_hat[2] << 0, 0, 1;

        
    }

    // 强制结构约束（正交化与归一化）
    // SVD寻找 Frobenius 范数下最近的正交矩阵
    void enforce_structure() {
        // 1. 强制 R_hat_B2I 正交化
        Eigen::JacobiSVD<Eigen::Matrix3d> svd_R(R_hat_B2I, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R_hat_B2I = svd_R.matrixU() * svd_R.matrixV().transpose();

        // 2. 强制 e_hat 正交化
        Eigen::Matrix3d E;
        E.col(0) = e_hat[0];
        E.col(1) = e_hat[1];
        E.col(2) = e_hat[2];

        Eigen::JacobiSVD<Eigen::Matrix3d> svd_E(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d E_orth = svd_E.matrixU() * svd_E.matrixV().transpose();

        e_hat[0] = E_orth.col(0);
        e_hat[1] = E_orth.col(1);
        e_hat[2] = E_orth.col(2);
    }
};

} 
#endif