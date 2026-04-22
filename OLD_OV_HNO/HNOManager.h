#ifndef HNO_MANAGER_H
#define HNO_MANAGER_H

#include <memory>
#include <vector>
#include <mutex>
#include <map>
#include <ros/ros.h>

#include "cam/CamBase.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "utils/sensor_data.h"

// Sub-modules
#include "HNOState.h"
#include "HNOPropagator.h"
#include "HNOUpdater.h"
#include "HNOFeature.h"
#include "HNOInitializer.h"

namespace ov_hno {

/**
 * @brief HNOManager 是整个里程计系统的核心管理者
 * 
 * 职责：
 * 1. 负责ROS节点的初始化、参数加载、话题订阅与发布。
 * 2. 接收IMU和相机数据，并分发给各子模块（Initializer, Propagator, Updater, Feature）。
 * 3. 协调系统状态（State）的流转。
 * 4. 进行可视化和Ground Truth验证。
 */
class HNOManager {
public:
    HNOManager(ros::NodeHandle& nh, const std::string& config_path);
    
    // 启动ROS订阅
    void launch_subscribers();

    // 测量值输入接口（外部调用或回调调用，需线程安全）
    void feed_measurement(const ov_core::ImuData& msg);
    void feed_measurement(const ov_core::CameraData& msg);
    
private:
    // --- 内部处理函数 ---
    
    // 加载配置文件（内参、外参、噪声等）
    void load_parameters(const std::string& config_path);
    
    // 加载真值数据用于评估（如果提供）
    void load_gt_data();
    
    // 处理相机数据的核心逻辑：特征追踪 -> 状态更新
    void process_camera_data(const ov_core::CameraData& msg);
    
    // 根据时间戳插值获取 GT (用于调试)
    bool get_interpolated_gt(double timestamp, Eigen::Vector3d& p_gt, Eigen::Matrix3d& R_gt);

    // --- 辅助函数 ---

    // 发布位姿、路径、TF
    void publish_state(double timestamp, const std::shared_ptr<HNOState>& state);
    
    // 发布特征点云、追踪图像
    void publish_visualization(double timestamp, const ov_core::CameraData& msg);
    
    // 计算并打印与真值的误差
    void compute_and_print_error(double timestamp, const Eigen::Vector3d& p_est, int num_feats, int num_obs);

    // --- ROS 回调 ---
    void imu_callback(const sensor_msgs::ImuConstPtr& msg);
    void stereo_callback(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1);

    // --- 成员变量 ---

    // ROS 句柄与发布器
    ros::NodeHandle nh_;
    ros::Publisher pub_pose;
    ros::Publisher pub_odom;
    ros::Publisher pub_path;
    ros::Publisher pub_feat;
    image_transport::Publisher pub_img;
    
    ros::Subscriber sub_imu;
    
    // 双目同步订阅机制
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_cam0;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_cam1;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::unique_ptr<Sync> sync;
    
    tf::TransformBroadcaster tf_broadcaster;
    nav_msgs::Path path_msg;

    // 子模块 (核心算法组件)
    std::shared_ptr<HNOState> state;             // 系统状态 (R, p, v, bg, ba)
    std::shared_ptr<HNOPropagator> propagator;   // IMU 积分器
    std::shared_ptr<HNOUpdater> updater;         // 状态更新器 (EKF/MSCKF逻辑)
    std::shared_ptr<HNOFeature> feature_handler; // 特征前端 (KLT + Triangulation)
    std::shared_ptr<HNOInitializer> initializer; // 初始化模块 (Static init)
    
    // 数据缓存
    std::mutex data_mutex;
    std::vector<ov_core::ImuData> imu_data_buffer;
    
    // 系统运行状态
    bool is_initialized = false;
    double current_time = -1;      // 当前估计状态的时间戳
    double first_timestamp = -1;   // 系统启动的时间戳
    double last_published_time = -1; // 限制发布频率
    
    // 相机配置
    std::vector<std::shared_ptr<ov_core::CamBase>> cams;
    std::vector<Eigen::Matrix4d> cams_T_C2B; // 外参: Camera -> Body (IMU)
    
    // Ground Truth (GT) 验证相关变量
    std::string path_gt;
    std::map<double, Eigen::VectorXd> gt_states;
    bool has_align = false;
    Eigen::Vector3d t_align = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();

    // --- Debug / Cheating Params ---
    bool use_gt_mapping = false;   // true: use GT for mapping (Cheating); false: use State (Real VIO)
};

} // namespace ov_hno

#endif // HNO_MANAGER_H
