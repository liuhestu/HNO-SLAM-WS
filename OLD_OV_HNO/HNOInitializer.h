#ifndef HNO_INITIALIZER_H
#define HNO_INITIALIZER_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "HNOState.h"

// OpenVINS Heads
#include "utils/sensor_data.h"

namespace ov_hno {

/**
 * OpenVINS有一个 ov_init::InertialInitializer 类
 * 但它深度绑定了OpenVINS自己的 State 类，包含复杂的协方差克隆管理
 * 直接调用它来初始化您的 HNOState 需要进行繁琐的数据结构转换
 * 对于静态初始化，核心逻辑非常简单:求均值 + 对齐重力,自己写
 */

class HNOInitializer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HNOInitializer();

    /**
     * @brief 添加 IMU 数据到初始化缓存
     * @param msg OpenVINS ImuData
     */
    void feedImuData(const ov_core::ImuData& msg);
    
    /**
     * @brief 尝试执行静态初始化
     * 计算静止状态下的平均加速度（用于重力对齐）和平均角速度（用于陀螺仪零偏）
     * 
     * @param state 需要填充初始值的状态对象
     * @param timestamp 初始化成功的时间戳
     * @return true 初始化成功，false 需要更多数据或不够静止
     */
    bool initialize(std::shared_ptr<HNOState> state, double& timestamp);

private:
    // 数据缓存
    std::vector<ov_core::ImuData> imu_buffer;
    
    // 最大窗口大小（避免无限增长）
    size_t window_size = 250; 

    // 静止检测阈值
    double max_acc_variance = 0.05; // m/s^2 (收紧阈值，原0.5)
    double max_gyro_variance = 0.01; // rad/s (收紧阈值，原0.1非常大)
};

}
#endif
