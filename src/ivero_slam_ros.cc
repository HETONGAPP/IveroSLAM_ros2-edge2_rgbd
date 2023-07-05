
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <iveroslam_ros2/srv/system_status.hpp>
#include <iveroslam_ros2/utils.h>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <System.h>

using std::placeholders::_1;
using std::placeholders::_2;

enum class State { Started, Stopped };

class RgbdSlamNode : public rclcpp::Node {
public:
  RgbdSlamNode(ORB_SLAM3::System* pSLAM) : Node("rgbdslam"), m_SLAM(pSLAM) {
    // create rgbd subscribers
    rgb_sub.subscribe(this, "/camera/color/image_raw");
    depth_sub.subscribe(this, "/camera/aligned_depth_to_color/image_raw");
    // create services
    start_service =
        create_service<std_srvs::srv::Trigger>("/ivero_slam/start", std::bind(&RgbdSlamNode::startSLAM, this, _1, _2));
    stop_service =
        create_service<std_srvs::srv::Trigger>("/ivero_slam/stop", std::bind(&RgbdSlamNode::stopSLAM, this, _1, _2));
    status_service = create_service<iveroslam_ros2::srv::SystemStatus>(
        "/ivero_slam/get_status", std::bind(&RgbdSlamNode::getStatus, this, _1, _2));
    cancel_service =
        create_service<std_srvs::srv::Trigger>("/ivero_slam/cancel", std::bind(&RgbdSlamNode::stopSLAM, this, _1, _2));
    // create publisher for image
    tracked_image_pub = create_publisher<sensor_msgs::msg::Image>("/ivero_slam/tracked_image", 10);
    cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/ivero_slam/map_points", 10);
    // create synchronizer for rgbd
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10), rgb_sub, depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::grabRGBD, this);
  }

  ~RgbdSlamNode() { m_SLAM->Shutdown(); }

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      approximate_sync_policy;

  void grabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD) {
    // output state message regularly
    static double previous_state_update = 0.0;
    auto rn = now().seconds();
    if (rn - previous_state_update > 0.5) {
      const auto state_str =
          current_state == State::Stopped ? "\033[1;31m Stopped\033[0m" : "\033[1;32m Started\033[0m";
      RCLCPP_INFO(this->get_logger(), "Current system state: %s", state_str);
      previous_state_update = rn;
    }
    // dont process if we are "stopped"
    if (current_state == State::Stopped) { return; }

    // proceed with normal slam processing
    try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    try {
      cv_ptrD = cv_bridge::toCvShare(msgD);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    double timestamp = msgRGB->header.stamp.sec + nsec_to_sec(msgRGB->header.stamp.nanosec);
    auto Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, timestamp);

    std_msgs::msg::Header header = msgRGB->header;

    // create rgb image with tracked points over it
    cv::Mat tracked_color = cv_ptrRGB->image.clone();
    cv::Scalar mapped_color(0, 255, 0);
    auto keypoints = m_SLAM->GetTrackedKeyPointsUn();
    for (const auto& kp : keypoints) {
      cv::Point tl(kp.pt.x - 3, kp.pt.y - 3);
      cv::Point br(kp.pt.x + 3, kp.pt.y + 3);
      cv::Point m(kp.pt.x, kp.pt.y);
      cv::circle(tracked_color, m, 0, mapped_color, 1);
      cv::rectangle(tracked_color, tl, br, mapped_color, 1);
    }
    const auto msg = cv_bridge::CvImage(header, "rgb8", tracked_color).toImageMsg();
    tracked_image_pub->publish(*msg.get());

    // publish point cloud
    const auto all_points = m_SLAM->GetFullPointCloud();
    const auto pointcloud_msg = points_to_cloud(all_points, timestamp);
    cloud_pub->publish(pointcloud_msg);
  }

  void startSLAM(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    current_state = State::Started;
    session_start_time = now().seconds();
    response->success = true;
    response->message = "Started";
  }

  void stopSLAM(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    current_state = State::Stopped;
    response->success = true;
    response->message = "Stopped";
  }

  void getStatus(const std::shared_ptr<iveroslam_ros2::srv::SystemStatus::Request> request,
                 std::shared_ptr<iveroslam_ros2::srv::SystemStatus::Response> response) {
    response->timestamp = now().seconds();
    response->session_start_time = session_start_time;
    response->tracking_status = m_SLAM->GetTrackingState();
    response->is_lost = m_SLAM->isLost();
    response->map_changed = m_SLAM->MapChanged();
    response->has_reset = m_SLAM->HasReset();
    if (current_state == State::Stopped) {
      response->started = false;
    } else {
      response->started = true;
    }
    response->success = true;
  }

  std::mutex mtx;
  ORB_SLAM3::System* m_SLAM;
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub;
  std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service;
  rclcpp::Service<iveroslam_ros2::srv::SystemStatus>::SharedPtr status_service;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracked_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;

  double session_start_time = -1.0;
  std::string output_folder_path;

  State current_state = State::Stopped;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ORB_SLAM3::System SLAM(get_vocab_path(), get_default_config_path(), ORB_SLAM3::System::RGBD, false, 0, get_date_string());
  auto node = std::make_shared<RgbdSlamNode>(&SLAM);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
