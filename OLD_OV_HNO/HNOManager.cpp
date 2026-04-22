#include "HNOManager.h"
#include <cam/CamRadtan.h>
#include <track/TrackKLT.h>
#include <cv_bridge/cv_bridge.h>
#include <utils/opencv_yaml_parse.h>

using namespace ov_hno;

/**
 * @brief 构造函数：初始化ROS句柄和各个子模块
 */
HNOManager::HNOManager(ros::NodeHandle& nh, const std::string& config_path) : nh_(nh) {
    
    // 1. 初始化核心状态与传播器
    state = std::make_shared<HNOState>();
    propagator = std::make_shared<HNOPropagator>();
    initializer = std::make_shared<HNOInitializer>();
    updater = std::make_shared<HNOUpdater>();
    
    // 2. 加载配置（内参、外参、噪声等）
    load_parameters(config_path);

    // 3. 将相机参数传递给前端
    feature_handler = std::make_shared<HNOFeature>(cams, cams_T_C2B);
    
    // 4. 将外参传递给更新器 (用于Updater中的几何计算)
    std::map<size_t, Eigen::Matrix4d> extrinsics_map;
    for(size_t i=0; i<cams_T_C2B.size(); i++) {
        extrinsics_map[i] = cams_T_C2B[i];
    }
    updater->setExtrinsics(extrinsics_map);
    
    // 5. 初始化ROS发布器
    // 姿态 (EKF输出)
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 100);
    // 路径 (轨迹)
    pub_path = nh.advertise<nav_msgs::Path>("path", 100);
    // 特征点云 (3D路标)
    pub_feat = nh.advertise<sensor_msgs::PointCloud>("features_3d", 100);
    
    // 图像传输 (带追踪轨迹的可视化图)
    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("image_track", 10);
    
    path_msg.header.frame_id = "world";

    ROS_INFO("[HNOManager] Initialized successfully.");
}

/**
 * @brief 从配置文件加载参数
 */
void HNOManager::load_parameters(const std::string& config_path) {
    ov_core::YamlParser parser(config_path);
    
    // --- 1. 相机参数 (Cam0, Cam1...) ---
    int num_cams = 2;
    parser.parse_config("max_cameras", num_cams);

    for(int i=0; i<num_cams; i++) {
        std::string cam_str = "cam" + std::to_string(i);
        
        std::vector<double> intrinsics, distortion_coeffs;
        std::vector<int> resolution;
        
        parser.parse_external("relative_config_imucam", cam_str, "intrinsics", intrinsics);
        parser.parse_external("relative_config_imucam", cam_str, "distortion_coeffs", distortion_coeffs);
        parser.parse_external("relative_config_imucam", cam_str, "resolution", resolution);

        auto cam = std::make_shared<ov_core::CamRadtan>(resolution[0], resolution[1]);
        Eigen::Matrix<double, 8, 1> calib;
        calib << intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3], 
                 distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3];
        cam->set_value(calib);
        cams.push_back(cam);
        
        // 解析外参 T_imu_cam (即 T_IC 或 T_B_C)
        Eigen::Matrix4d T_in_file;
        parser.parse_external("relative_config_imucam", cam_str, "T_imu_cam", T_in_file);
        
        // 这里的 T_in_file 就是 T_B_C (Body to Camera frame transformation?? 或者是 T_I_C)
        // 通常 T_imu_cam 表示 p_imu = T * p_cam
        // 如果我们要在 HNOFeature 里用 T_B_C (p_body = T * p_cam)，那么应该直接用 T_in_file
        // 但是之前的注释写着 "HNOUpdater 期望的是 T_cam_imu"，这有点乱。
        // 关键是看 HNOFeature 里面: p_body = R_bc * p_c_left + p_bc
        // 这说明 HNOFeature 认为传入的是 T_B_C
        
        // 如果配置文件里 T_imu_cam 是 T_I_C (Standard VINS format, p_I = T * p_C)
        // 那么 T_C2B 应该直接等于 T_in_file
        
        cams_T_C2B.push_back(T_in_file); 
    }

    // --- 2. IMU 噪声参数 ---
    double acc_sigma = 0.01;
    double gyro_sigma = 0.001;
    parser.parse_config("accelerometer_noise_density", acc_sigma);
    parser.parse_config("gyroscope_noise_density", gyro_sigma);

    HNOPropagator::NoiseParams noise_params;
    noise_params.noise_acc = acc_sigma;
    noise_params.noise_gyro = gyro_sigma;
    propagator->setNoiseParams(noise_params);
    
    // --- 3. Ground Truth ---
    // 尝试从ROS参数服务器获取GT路径，如果没有则尝试从配置文件或默认参数获取
    if (nh_.getParam("path_gt", path_gt)) {
        ROS_INFO("Found private param path_gt: %s", path_gt.c_str());
    } else if (nh_.getParam("/run_subscribe_hno/path_gt", path_gt)) {
        ROS_INFO("Found global param path_gt: %s", path_gt.c_str());
    }
    
    if (!path_gt.empty()) {
        load_gt_data();
    } else {
        ROS_WARN("No GT path provided in params.");
    }
    
    // --- 4. 调试/Cheat 模式参数 ---
    // 默认开启 GT 建图 (Cheating Mode) 用于调试算法
    nh_.param<bool>("use_gt_mapping", use_gt_mapping, true);
    if(use_gt_mapping) {
        ROS_WARN("[HNO] RUNNING IN CHEAT MODE: Using GT for Mapping!");
    } else {
        ROS_INFO("[HNO] RUNNING IN REAL MODE: Using Estimation for Mapping.");
    }
}

/**
 * @brief 启动ROS订阅器
 */
void HNOManager::launch_subscribers() {
    // IMU 订阅
    std::string topic_imu;
    nh_.param<std::string>("topic_imu", topic_imu, "/imu0");
    sub_imu = nh_.subscribe(topic_imu, 1000, &HNOManager::imu_callback, this);
    
    // 相机订阅 (单目/双目)
    int num_cams = 2;
    nh_.param("num_cams", num_cams, 2);
    
    std::string topic_cam0, topic_cam1;
    nh_.param<std::string>("topic_cam0", topic_cam0, "/cam0/image_raw");
    nh_.param<std::string>("topic_cam1", topic_cam1, "/cam1/image_raw");

    if (num_cams == 2) {
        sub_cam0 = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
            new message_filters::Subscriber<sensor_msgs::Image>(nh_, topic_cam0, 10));
        sub_cam1 = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
            new message_filters::Subscriber<sensor_msgs::Image>(nh_, topic_cam1, 10));
            
        // 使用 ExactTime 同步策略
        sync = std::unique_ptr<Sync>(new Sync(MySyncPolicy(10), *sub_cam0, *sub_cam1));
        sync->registerCallback(boost::bind(&HNOManager::stereo_callback, this, _1, _2));
        ROS_INFO("Subscribed to Stereo: %s, %s", topic_cam0.c_str(), topic_cam1.c_str());
    } else {
        ROS_ERROR("Only stereo configuration (num_cams=2) is supported in this implementation.");
        ros::shutdown();
    }
}

// --- ROS Callbacks ---

void HNOManager::imu_callback(const sensor_msgs::ImuConstPtr& msg) {
    ov_core::ImuData data;
    data.timestamp = msg->header.stamp.toSec();
    data.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    data.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    feed_measurement(data);
}

void HNOManager::stereo_callback(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1) {
    ov_core::CameraData data;
    data.timestamp = msg0->header.stamp.toSec();
    data.sensor_ids = {0, 1};
    try {
        data.images.push_back(cv_bridge::toCvCopy(msg0, "mono8")->image);
        data.images.push_back(cv_bridge::toCvCopy(msg1, "mono8")->image);
        data.masks.push_back(cv::Mat::zeros(data.images[0].rows, data.images[0].cols, CV_8UC1));
        data.masks.push_back(cv::Mat::zeros(data.images[1].rows, data.images[1].cols, CV_8UC1));
    } catch (...) { return; }
    feed_measurement(data);
}

// --- Core Logic ---

/**
 * @brief 处理IMU数据：缓存、初始化检查、状态积分、高频发布
 */
void HNOManager::feed_measurement(const ov_core::ImuData& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // 1. 始终存入 buffer
    imu_data_buffer.push_back(msg);

    // 2. 如果尚未初始化，尝试静态初始化
    if(!is_initialized) {
        initializer->feedImuData(msg);
        if(initializer->initialize(state, current_time)) {
            // [Phase 1 Cheat Init]
            // 强制使用 GT 覆盖初始状态，排除初始化算法的干扰！
            Eigen::Vector3d p_gt_init;
            Eigen::Matrix3d R_gt_init;
            if(get_interpolated_gt(current_time, p_gt_init, R_gt_init)) {
                ROS_WARN("[HNO] Force overwriting initialization with GT!");
                state->p_hat = p_gt_init;
                state->R_hat_B2I = R_gt_init;
                // 速度设为 GT 的差分或者 0? 暂时设为 0
                state->v_hat.setZero();
                // 零偏保持初始化算出来的或者设为 0
                // state->bg.setZero();
                // state->ba.setZero();
            }
            
            is_initialized = true;
            ROS_INFO("Statical Initialization Done at %.3f", current_time);
        }
        return; // 初始化未完成不进行后续处理
    }

    // 3. 高频传播与发布 (Visualization Propagate)
    // 这是一个独立的 Propagate 过程，仅仅为了高频输出，不更新系统的主状态 `state`
    auto state_viz = std::make_shared<HNOState>(*state); // 拷贝当前状态
    double sim_time = current_time;
    
    for(const auto& imu : imu_data_buffer) {
        if(imu.timestamp > sim_time) {
             double dt = imu.timestamp - sim_time;
             if(dt > 1e-6) {
                 propagator->propagate(state_viz, imu.wm, imu.am, dt);
                 sim_time = imu.timestamp;
             }
        }
    }

    // 4. 发布高频位姿
    publish_state(sim_time, state_viz);
}

/**
 * @brief 处理相机数据：核心VIO流程
 */
void HNOManager::feed_measurement(const ov_core::CameraData& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    if(is_initialized) {
        process_camera_data(msg);
    }
}

bool HNOManager::get_interpolated_gt(double timestamp, Eigen::Vector3d& p_gt, Eigen::Matrix3d& R_gt) {
    if (gt_states.empty()) return false;
    
    // 找到第一个大于等于 timestamp 的迭代器
    auto it = gt_states.lower_bound(timestamp);
    if (it == gt_states.end() || it == gt_states.begin()) return false;
    
    auto it_prev = std::prev(it);
    double t1 = it_prev->first;
    double t2 = it->first;
    double alpha = (timestamp - t1) / (t2 - t1); // 线性插值系数
    
    // 插值位置
    Eigen::Vector3d p1 = it_prev->second.head<3>();
    Eigen::Vector3d p2 = it->second.head<3>();
    p_gt = (1.0 - alpha) * p1 + alpha * p2;
    
    // 插值姿态 (Slerp)
    Eigen::Quaterniond q1(it_prev->second(6), it_prev->second(3), it_prev->second(4), it_prev->second(5));
    Eigen::Quaterniond q2(it->second(6), it->second(3), it->second(4), it->second(5));
    R_gt = q1.slerp(alpha, q2).toRotationMatrix();
    
    return true;
}

void HNOManager::process_camera_data(const ov_core::CameraData& msg) {
    if(msg.timestamp <= current_time) {
        ROS_WARN("Camera message skipped (old params): %.3f <= %.3f", msg.timestamp, current_time);
        return;
    }

    // --- 1. IMU 积分传播 (Propagate) ---
    // 将系统状态从上一帧时刻 current_time 推进到当前图像时刻 msg.timestamp
    for(const auto& imu : imu_data_buffer) {
        if(imu.timestamp > current_time && imu.timestamp <= msg.timestamp) {
             double dt = imu.timestamp - current_time;
             if(dt > 1e-6) propagator->propagate(state, imu.wm, imu.am, dt);
             current_time = imu.timestamp;
        }
    }
    // 处理两个IMU帧之间的小数时间差
    if(msg.timestamp > current_time && !imu_data_buffer.empty()) {
         auto& last_imu = imu_data_buffer.back();
         double dt = msg.timestamp - current_time;
         if(dt > 1e-6) propagator->propagate(state, last_imu.wm, last_imu.am, dt);
         current_time = msg.timestamp;
    }

    // 清除过期的IMU数据 (保留 msg.timestamp 之后的作为下一次的开始)
    auto it = imu_data_buffer.begin();
    while(it != imu_data_buffer.end() && it->timestamp <= current_time) {
        it = imu_data_buffer.erase(it);
    }
    
    // --- 2. 特征追踪 (Feature Tracking) ---
    // 调用 Feature Handler 获取有效的观测数据
    std::vector<HNOObservation> valid_measurements;

    // --- 获取真值 (Cheating Mode) ---
    Eigen::Vector3d p_gt;
    Eigen::Matrix3d R_gt;
    bool has_gt = get_interpolated_gt(msg.timestamp, p_gt, R_gt);

    // 设置初始时间戳
    if (first_timestamp < 0) first_timestamp = msg.timestamp;

    // [Mode Selection: GT Cheat or Normal VIO]
    // 强制使用 GT 初始化，如果找不到 GT 则正常初始化
    // 这样能解决初始化发散问题，让我们专注调试后面的建图
    if( !is_initialized ) {
        return; // Initialization is handled in feed_measurement(imu)
    }

    // 根据参数选择建图模式
    // 注意：如果有 GT 存在且开启了 Cheat 模式，则使用 GT
    // 否则（无GT或关闭Cheat），使用 Estimate
    bool enable_cheat = use_gt_mapping && has_gt;

    static bool phase2_started = false; 

    if (enable_cheat) {
        // [Phase 1: Cheat Mode]
        ROS_INFO_THROTTLE(1.0, "[Phase 1] Using GT for Mapping (Cheat Mode)");
        feature_handler->feed_measurement(
            msg, 
            state->R_hat_B2I, state->p_hat, // 给 RANSAC/Update 用的估计值
            R_gt, p_gt,                     // 给 建图 用的真值 (Feature用这个做三角化/校验)
            valid_measurements
        );
    } else {
        // [Phase 2: Estimate Mode]
        ROS_INFO_THROTTLE(1.0, "[Phase 2] Using ESTIMATE for Mapping (Real Mode)");

        feature_handler->feed_measurement(
            msg, 
            state->R_hat_B2I, state->p_hat, // 给 RANSAC
            state->R_hat_B2I, state->p_hat, // 给 建图也用估计值
            valid_measurements
        );
    }
    
    // --- 3. 状态更新 (Update / Correction) ---
    // 如果有有效的特征观测，使用 MSCKF/EKF 更新状态
    int num_obs = valid_measurements.size();
    if(!valid_measurements.empty()) {
        updater->update(state, valid_measurements);
    }
    
    // --- 4. 结果输出与可视化 ---
    
    // 计算误差
    int map_size = feature_handler->get_active_map().size();
    compute_and_print_error(msg.timestamp, state->p_hat, map_size, num_obs);
    
    // 发布可视化话题
    publish_visualization(msg.timestamp, msg);
}


// --- Helper Functions ---

void HNOManager::publish_state(double timestamp, const std::shared_ptr<HNOState>& state_viz) {
    ros::Time rtime(timestamp);

    // 防止重复时间戳发布
    if (std::abs(timestamp - last_published_time) < 1e-9) return;
    last_published_time = timestamp;
    
    // 1. Pose
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.stamp = rtime;
    msg_pose.header.frame_id = "world";
    msg_pose.pose.pose.position.x = state_viz->p_hat.x();
    msg_pose.pose.pose.position.y = state_viz->p_hat.y();
    msg_pose.pose.pose.position.z = state_viz->p_hat.z();
    Eigen::Quaterniond q(state_viz->R_hat_B2I);
    msg_pose.pose.pose.orientation.w = q.w();
    msg_pose.pose.pose.orientation.x = q.x();
    msg_pose.pose.pose.orientation.y = q.y();
    msg_pose.pose.pose.orientation.z = q.z();
    pub_pose.publish(msg_pose);
    
    // 2. Path
    geometry_msgs::PoseStamped ps;
    ps.header = msg_pose.header;
    ps.pose = msg_pose.pose.pose;
    path_msg.header.stamp = rtime;
    path_msg.poses.push_back(ps);
    pub_path.publish(path_msg);
    
    // 3. TF
    tf::Transform transform;
    tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    transform.setOrigin(tf::Vector3(state_viz->p_hat.x(), state_viz->p_hat.y(), state_viz->p_hat.z()));
    transform.setRotation(tf_q);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, rtime, "world", "imu"));
}

void HNOManager::publish_visualization(double timestamp, const ov_core::CameraData& msg) {
    ros::Time rtime(timestamp);

    // 1. 发布特征点云 (Feature Map)
    // 这里的点云是 Triangulation 后的 3D 坐标
    sensor_msgs::PointCloud msg_pc;
    msg_pc.header.stamp = rtime;
    msg_pc.header.frame_id = "world";
    
    const auto& map = feature_handler->get_active_map();
    for(auto& pair : map) {
        geometry_msgs::Point32 p;
        p.x = pair.second.x(); p.y = pair.second.y(); p.z = pair.second.z();
        msg_pc.points.push_back(p);
    }
    pub_feat.publish(msg_pc);

    // 2. 发布追踪图像 (Image Track)
    if(pub_img.getNumSubscribers() > 0) {
        
        // --- 绘制特征点状态：只为红色成熟点画轨迹，黄色不画轨迹 ---
        const auto& active_map = feature_handler->get_active_map();
        auto obs = feature_handler->get_tracker()->get_last_obs();
        auto ids = feature_handler->get_tracker()->get_last_ids();

        // 构造允许画轨迹的 ID 集合（成熟点）
        std::unordered_set<size_t> allowed_trails;
        for(const auto& kv : active_map) allowed_trails.insert(kv.first);

        // 让 display_history 直接生成左右拼接的可视化（内部会根据缓存的左右图尺寸自动排版）
        cv::Mat img_viz;
        feature_handler->get_tracker()->display_history(img_viz, 0, 255, 0, 255, 0, 0, {}, "HNO Tracker", &allowed_trails);

        // 如果没有生成图像，直接跳过
        if(img_viz.empty()) return;

        // 基于拼接图像划分左右 ROI，再叠加当前观测的红/黄标记
        int width = msg.images[0].cols;
        int height = msg.images[0].rows;
        bool has_right_img = (msg.images.size() > 1 && !msg.images[1].empty());
        if(img_viz.cols >= width && img_viz.rows >= height) {
            cv::Mat left_roi = img_viz(cv::Rect(0, 0, width, height));
            cv::Mat right_roi;
            if(has_right_img && img_viz.cols >= 2*width) {
                right_roi = img_viz(cv::Rect(width, 0, width, height));
            }

            // 左目标记
            if(obs.count(0) && ids.count(0)) {
                size_t num = ids[0].size();
                for(size_t i=0; i<num; ++i) {
                    size_t id = ids[0][i];
                    cv::Point2f pt = obs[0][i].pt;
                    if(active_map.count(id)) {
                        cv::circle(left_roi, pt, 6, cv::Scalar(0, 0, 255), 2); // 红色成熟点
                    } else {
                        cv::circle(left_roi, pt, 6, cv::Scalar(0, 255, 255), 1); // 黄色其他点
                    }
                }
            }

            // 右目标记（画在拼接图右半部分）
            if(has_right_img && !right_roi.empty() && obs.count(1) && ids.count(1)) {
                for(size_t k=0; k<ids[1].size(); ++k) {
                    size_t id = ids[1][k];
                    cv::Point2f pt = obs[1][k].pt;
                    if(active_map.count(id)) {
                        cv::circle(right_roi, pt, 6, cv::Scalar(0, 0, 255), 2);
                    } else {
                        cv::circle(right_roi, pt, 6, cv::Scalar(0, 255, 255), 1);
                    }
                }
            }
        }

        sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_viz).toImageMsg();
        msg_img->header.stamp = rtime;
        pub_img.publish(msg_img);
    }
}

void HNOManager::load_gt_data() {
    std::ifstream f(path_gt);
    if(!f.is_open()) {
        ROS_ERROR("Failed to open GT file: %s", path_gt.c_str());
        return;
    }
    
    std::string line;
    while(std::getline(f, line)) {
        if(line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::vector<double> val;
        double d;
        while(ss >> d) val.push_back(d);

        if(val.size() >= 8) {
            double ts = val[0];
            // 简单的时间单位转换 (ns -> s)
            if (ts > 1e12) ts *= 1e-9;
            else if (ts > 1e10) ts *= 1e-9;
            
            Eigen::VectorXd pose(7);
            pose << val[1], val[2], val[3], val[4], val[5], val[6], val[7];
            gt_states[ts] = pose;
        }
    }
    ROS_INFO("Loaded %zu GT poses.", gt_states.size());
}

void HNOManager::compute_and_print_error(double timestamp, const Eigen::Vector3d& p_est, int num_feats, int num_obs) {
    
    std::string err_str = "";
    
    if(!gt_states.empty()) {
        auto it = gt_states.lower_bound(timestamp);
        bool found = false;
        Eigen::VectorXd best_gt;
        
        double min_dt = 100.0;
        
        // Check current
        if (it != gt_states.end()) {
            double dt = std::abs(it->first - timestamp);
            if (dt < min_dt) { min_dt = dt; best_gt = it->second; found = true; }
        }
        // Check prev
        if (it != gt_states.begin()) {
            auto it_prev = std::prev(it);
            double dt = std::abs(it_prev->first - timestamp);
            if (dt < min_dt) { min_dt = dt; best_gt = it_prev->second; found = true; }
        }

        if(found && min_dt < 0.05) { // 允许 50ms 误差
            Eigen::Vector3d p_gt = best_gt.head<3>();
            
            // 如果还未对齐，进行首次对齐
            if(!has_align) { 
                // 只对齐平移和Yaw? 还是全对齐?
                // 这里简单地对齐平移 + 旋转
                Eigen::Quaterniond q_gt(best_gt(6), best_gt(3), best_gt(4), best_gt(5));
                Eigen::Matrix3d R_gt = q_gt.toRotationMatrix();
                // R_align = R_gt * R_est^T
                R_align = R_gt * state->R_hat_B2I.transpose();
                // t_align = p_gt - R_align * p_est
                t_align = p_gt - R_align * p_est; 
                has_align = true; 
            }
            
            Eigen::Vector3d p_aligned = R_align * p_est + t_align;
            Eigen::Vector3d err = p_aligned - p_gt;
            char buff[128];
            snprintf(buff, sizeof(buff), "Err:%.3f (xyz: %.2f %.2f %.2f)", err.norm(), err.x(), err.y(), err.z());
            err_str = std::string(buff);
        } else {
            // GT 同步丢失
            static double last_warn = 0;
            if(timestamp - last_warn > 1.0) {
                // printf("[HNO] No sync GT found (min_dt=%.3f)\n", min_dt);
                last_warn = timestamp;
            }
        }
    }

    Eigen::Vector3d v = state->v_hat;
    Eigen::Vector3d bg = state->bg;
    Eigen::Vector3d ba = state->ba;
    
    // [Time] [MapSize/ObsNum] [Pos] [Vel] [BiasG] [Err]
    printf("[HNO] %.3f | Feat:%d/%d | Pos:%.2f %.2f %.2f | Vel:%.2f %.2f %.2f | %s\n",
        timestamp, num_feats, num_obs,
        p_est.x(), p_est.y(), p_est.z(),
        v.x(), v.y(), v.z(),
        err_str.c_str()
    );
}
