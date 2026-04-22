#include "HNOInitializer.h"
#include <iostream>
#include <numeric>

namespace ov_hno {

HNOInitializer::HNOInitializer() {}

void HNOInitializer::feedImuData(const ov_core::ImuData& msg) {
    imu_buffer.push_back(msg);
    // 滑动窗口：如果超过大小，丢弃最旧的数据
    if (imu_buffer.size() > window_size) {
        imu_buffer.erase(imu_buffer.begin());
    }
}

bool HNOInitializer::initialize(std::shared_ptr<HNOState> state, double& timestamp) {
    // 1. 数据充足性检查
    if(imu_buffer.size() < window_size) return false; 
    
    // 2. 计算均值
    Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
    
    for(const auto& d : imu_buffer) {
        sum_acc += d.am;
        sum_gyro += d.wm; 
    }
    
    Eigen::Vector3d ave_acc = sum_acc / imu_buffer.size();
    Eigen::Vector3d ave_gyro = sum_gyro / imu_buffer.size(); 
    
    // 3. 计算静止方差
    Eigen::Vector3d var_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d var_gyro = Eigen::Vector3d::Zero();
    
    for(const auto& d : imu_buffer) {
        var_acc += (d.am - ave_acc).cwiseAbs2();
        var_gyro += (d.wm - ave_gyro).cwiseAbs2();
    }
    var_acc /= imu_buffer.size();
    var_gyro /= imu_buffer.size();

    // 静止检测，如果数据晃动太大，说明不是静止状态，不能用于初始化
    if(var_acc.norm() > max_acc_variance || var_gyro.norm() > max_gyro_variance) {
        std::cout << "[HNOInit] Waiting for stationary... Var Acc: " << var_acc.norm() 
                  << " Gyro: " << var_gyro.norm() << std::endl;
        return false;
    }

    // 4. 零偏初始化
    // 陀螺仪零偏直接取均值 (因为静止时真实角速度为0)
    Eigen::Vector3d bg0 = ave_gyro;
    
    // 静态初始化通常无法分离重力和加速度计零偏，设为0
    Eigen::Vector3d ba0 = Eigen::Vector3d::Zero();

    // 5. 重力对齐 (Gravity Alignment)
    /**
     * @brief 寻找初始姿态 R_B2I (Body to Inertial)
     * IMU模型: a_m = R_B2I^T * (a_I - g_I) + b_a + n_a
     * 静止时 a_I = 0, b_a ≈ 0 (或者已经校准), n_a 均值为0
     * 则 a_m = - R_B2I^T * g_I
     *    R_B2I * a_m = -g_I
     * 设 Inertial 系重力 g_I = [0, 0, -9.81]
     * 则 R_B2I * a_m ≈ [0, 0, 9.81]
     * 目标: 寻找 R_B2I 使得 a_m 旋转后指向 Z 轴正向 [0,0,1]
     */

    Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();     // [0,0,1]
    Eigen::Vector3d acc_dir = ave_acc.normalized();     // 机体系下归一化测量值

    // Debug print
    std::cout << "[HNOInit] Average Acc: " << ave_acc.transpose() << std::endl;
    std::cout << "[HNOInit] Acc Dir: " << acc_dir.transpose() << std::endl;
    
    // Quaternion::FromTwoVectors(u, v) creates rotation that maps u to v
    // We want R * acc_dir = z_axis  => is matching FromTwoVectors definition
    Eigen::Quaterniond R0_q_B2I = Eigen::Quaterniond::FromTwoVectors(acc_dir, z_axis);
    
    // Debug print RPY
    Eigen::Vector3d rpy_init = R0_q_B2I.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "[HNOInit] Init RPY (rad): " << rpy_init.transpose() << std::endl;
    std::cout << "[HNOInit] Init RPY (deg): " << rpy_init.transpose() * 180.0 / M_PI << std::endl;

    // 我们可能需要加上一个设定好的初始偏航 (Yaw)
    // 通常我们假设初始 Yaw 为 0, 但 FromTwoVectors 产生的旋转可能包含任意 Yaw
    // 这里我们简单地使用 FromTwoVectors 的结果，它会找到"最小旋转"
    
    Eigen::Matrix3d R0_B2I = R0_q_B2I.toRotationMatrix(); 

    
    // 6. 填充状态
    state->initialize(R0_B2I, 
                      Eigen::Vector3d::Zero(), // p0
                      Eigen::Vector3d::Zero(), // v0
                      bg0, 
                      ba0);
    
    timestamp = imu_buffer.back().timestamp;
    
    std::cout << "[HNOInit] Initialized Success!" << std::endl;
    std::cout << "  Init Rotation (RPY): " << R0_B2I.eulerAngles(2,1,0).transpose() * 180.0/M_PI << std::endl;
    std::cout << "  Init Gyro Bias: " << bg0.transpose() << std::endl;
    std::cout << "  Init Acc Bias: " << ba0.transpose() << std::endl;

    // 清空缓存
    imu_buffer.clear();
    
    return true;
}

}
