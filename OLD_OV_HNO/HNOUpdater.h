#ifndef HNO_UPDATER_H
#define HNO_UPDATER_H

#include "HNOState.h"
#include <vector>
#include <memory>
#include <map>
#include <Eigen/Dense>

// OpenVINS Heads
#include "feat/FeatureDatabase.h"
#include "feat/Feature.h"

namespace ov_hno {

// 定义单个特征点的观测数据结构 (From Reference)
struct HNOObservation {
    // 归一化观测向量 (x, y, 1).normalized()
    Eigen::Vector3d uv_left;
    Eigen::Vector3d uv_right;
    bool isValidRight;
    // 世界系下的 3D 坐标
    Eigen::Vector3d xyz;
};

class HNOUpdater {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 构造函数，只在程序启动、创建这个类对象的时候运行一次
    HNOUpdater();

    /**
     * @brief 设置相机外参 (Camera -> Body)
     * @param T_C2B_map [R_C2B p_C2B] 外参字典，key为相机索引 (0: 左目left, 1: 右目right)
     */
    void setExtrinsics(const std::map<size_t, Eigen::Matrix4d>& T_C2B_map);

    /**
     * @brief HNO 更新接口
     * 接收预处理好的特征点列表（包含世界坐标和归一化观测），执行更新
     * @param state 当前 HNO 状态
     * @param observations 预处理好的有效观测列表
     */
    void update(std::shared_ptr<HNOState> state,
                const std::vector<HNOObservation>& observations);

private:
    // 参数配置
    double pixel_noise = 2.0;    // 重投影误差/像素噪声 (pixel)
    double focal_length = 460.0; // 焦距 (pixel)

    // 计算投影算子 pi(x) = I - x*x^T
    Eigen::Matrix3d project_pi(const Eigen::Vector3d& x);

    // 外参: R_C2B 从相机旋转到机体，pc相机原点在机体系下的坐标
    Eigen::Matrix3d R_C2B_left, R_C2B_right;
    Eigen::Vector3d pc_left, pc_right;
    bool has_stereo_extrinsics = false;
};

} 
#endif


