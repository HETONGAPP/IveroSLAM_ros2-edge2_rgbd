
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <iveroslam_ros2/utils.h>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <zipper/zipper.h>

using std::placeholders::_1;
using std::placeholders::_2;

class DataRecorder : public rclcpp::Node {
public:
  DataRecorder() : Node("data_recorder") {
    // create rgbd subscribers
    rgb_sub.subscribe(this, "/camera/color/image_raw");
    depth_sub.subscribe(this, "/camera/aligned_depth_to_color/image_raw");
    // create services
    start_service = create_service<std_srvs::srv::Trigger>("/data_recorder/start",
                                                           std::bind(&DataRecorder::startScan, this, _1, _2));
    stop_service =
        create_service<std_srvs::srv::Trigger>("/data_recorder/stop", std::bind(&DataRecorder::stopScan, this, _1, _2));
    cancel_service = create_service<std_srvs::srv::Trigger>("/data_recorder/cancel",
                                                            std::bind(&DataRecorder::cancelScan, this, _1, _2));
    // rgbd synchronizer
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10), rgb_sub, depth_sub);
    syncApproximate->registerCallback(&DataRecorder::outputRGBD, this);
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      approximate_sync_policy;

  void outputRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD) {
    if (!recording) { return; }

    // convert images to opencv and output to file
    try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    try {
      cv_ptrD = cv_bridge::toCvShare(msgD, "16UC1");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    rclcpp::Time ros_time = msgRGB->header.stamp;
    const auto filename = std::to_string(ros_time.nanoseconds());
    cv::imwrite(output_folder + "/rgb/" + filename + ".jpg", cv_ptrRGB->image);
    cv::imwrite(output_folder + "/depth/" + filename + ".png", cv_ptrD->image);
  }

  void startScan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::unique_lock<std::mutex> lk(mtx);
    output_folder = setup_output_folders(get_date_string(), get_default_config_path());
    recording = true;
    response->success = true;
    response->message = output_folder;
    lk.unlock();
  }

  void stopScan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::unique_lock<std::mutex> lk(mtx);
    recording = false;

    auto output_folder_zip = output_folder + ".zip";
    zipper::Zipper zipper(output_folder_zip);
    zipper.add(output_folder, zipper::Zipper::zipFlags::Faster);
    zipper.close();

    RCLCPP_WARN(this->get_logger(), "Deleting folder: %s", output_folder.c_str());
    boost::system::error_code ec;
    boost::filesystem::remove_all(output_folder, ec);
    if (ec) { std::cerr << "Error deleting directory: " << ec.message() << std::endl; }

    response->success = true;
    response->message = output_folder_zip;
    lk.unlock();
  }

  void cancelScan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::unique_lock<std::mutex> lk(mtx);
    recording = false;

    RCLCPP_WARN(this->get_logger(), "Deleting folder: %s", output_folder.c_str());
    boost::system::error_code ec;
    boost::filesystem::remove_all(output_folder, ec);
    if (ec) { std::cerr << "Error deleting directory: " << ec.message() << std::endl; }

    response->success = true;
    response->message = "Deleted folder: " + output_folder;
    lk.unlock();
  }

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub;
  std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;
  std::mutex mtx;

  std::string output_folder;
  bool recording;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<DataRecorder>());
  rclcpp::shutdown();
  return 0;
}