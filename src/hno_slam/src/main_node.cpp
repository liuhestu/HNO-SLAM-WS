#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "System.h"

namespace hno_slam {

class RosBridge {
 public:
  RosBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    std::string config_path;
    std::string topic_imu;
    std::string topic_cam0;
    std::string topic_cam1;

    pnh_.param<std::string>("config_path", config_path, "");
    pnh_.param<std::string>("topic_imu", topic_imu, "/imu0");
    pnh_.param<std::string>("topic_cam0", topic_cam0, "/cam0/image_raw");
    pnh_.param<std::string>("topic_cam1", topic_cam1, "/cam1/image_raw");

    system_ = std::make_shared<System>(config_path);

    pub_path_ = nh_.advertise<nav_msgs::Path>("path", 100);
    pub_img_track_ = nh_.advertise<sensor_msgs::Image>("image_track", 10);
    path_msg_.header.frame_id = "world";

    imu_sub_ = nh_.subscribe(topic_imu, 4000, &RosBridge::imu_callback, this);

    left_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, topic_cam0, 20);
    right_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, topic_cam1, 20);
    sync_ = std::make_unique<message_filters::Synchronizer<StereoPolicy>>(StereoPolicy(50), *left_sub_, *right_sub_);
    sync_->registerCallback(&RosBridge::stereo_callback, this);
  }

 private:
  using StereoPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;

  static cv::Mat to_gray_image(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    if (cv_ptr->image.channels() == 1) {
      return cv_ptr->image.clone();
    }
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }

  void imu_callback(const sensor_msgs::ImuConstPtr& msg) {
    ov_core::ImuData imu;
    imu.timestamp = msg->header.stamp.toSec();
    imu.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    system_->feed_imu(imu);
  }

  void stereo_callback(const sensor_msgs::ImageConstPtr& left,
                       const sensor_msgs::ImageConstPtr& right) {
    ov_core::CameraData cam;
    cam.timestamp = left->header.stamp.toSec();
    cam.sensor_ids = {0, 1};
    cam.images = {to_gray_image(left), to_gray_image(right)};
    cam.masks = {
      cv::Mat::zeros(cam.images[0].rows, cam.images[0].cols, CV_8UC1),
      cv::Mat::zeros(cam.images[1].rows, cam.images[1].cols, CV_8UC1)};
    system_->feed_camera(cam);

    auto state = system_->get_state();
    if (state) {
      geometry_msgs::PoseStamped ps;
      ps.header.stamp = left->header.stamp;
      ps.header.frame_id = "world";
      ps.pose.position.x = state->p_hat.x();
      ps.pose.position.y = state->p_hat.y();
      ps.pose.position.z = state->p_hat.z();

      Eigen::Quaterniond q(state->R_hat);
      ps.pose.orientation.w = q.w();
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();

      path_msg_.header.stamp = ps.header.stamp;
      path_msg_.poses.push_back(ps);
      pub_path_.publish(path_msg_);
    }

    cv::Mat viz;
    cv_bridge::CvImage out;
    out.header = left->header;
    if (system_->get_track_visualization(viz) && !viz.empty()) {
      out.image = viz;
      out.encoding = (viz.channels() == 1) ? "mono8" : "bgr8";
    } else {
      out.image = cam.images[0];
      out.encoding = "mono8";
    }
    pub_img_track_.publish(out.toImageMsg());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::shared_ptr<System> system_;
  ros::Subscriber imu_sub_;
  ros::Publisher pub_path_;
  ros::Publisher pub_img_track_;
  nav_msgs::Path path_msg_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> right_sub_;
  std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> sync_;
};

}  // namespace hno_slam

int main(int argc, char** argv) {
  ros::init(argc, argv, "hno_slam_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  hno_slam::RosBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
