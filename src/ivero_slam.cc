
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <zipper/zipper.h>

#include <iveroslam_ros2/CallbackHarness.h>
#include <iveroslam_ros2/msg/system_status.hpp>
#include <iveroslam_ros2/srv/system_status.hpp>
#include <iveroslam_ros2/utils.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <System.h>

enum class State { Started, Stopped };

State current_state = State::Stopped;
bool output_results = true;
bool b_continue_session;
double session_start_time = -1.0;
iveroslam_ros2::msg::SystemStatus current_status;
std::mutex mtx;

void exit_loop_handler(int s) {
  std::cout << "\nExiting IveroSLAM" << std::endl;
  b_continue_session = false;
}

void start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  std::string response_msg;
  current_state = State::Started;
  output_results = true;
  response_msg = "Started";
  response->success = true;
  response->message = response_msg;
}

void stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  std::string response_msg;
  current_state = State::Stopped;
  output_results = true;
  response_msg = "Stopped";
  response->success = true;
  response->message = response_msg;
}

void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  std::string response_msg;
  current_state = State::Stopped;
  output_results = false;
  response_msg = "Canceled";
  response->success = true;
  response->message = response_msg;
}

void get_status(const std::shared_ptr<iveroslam_ros2::srv::SystemStatus::Request> request,
                std::shared_ptr<iveroslam_ros2::srv::SystemStatus::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  rclcpp::Time stamp = current_status.header.stamp;
  response->timestamp = stamp.seconds();
  response->session_start_time = session_start_time;
  response->tracking_status = current_status.tracking_status;
  response->is_lost = current_status.is_lost;
  response->map_changed = current_status.map_changed;
  if (current_state == State::Stopped) {
    response->started = false;
  } else {
    response->started = true;
  }
  response->success = true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // ros things
  auto node = rclcpp::Node::make_shared("ivero_slam_node");

  // services for controlling the state
  auto start_service = node->create_service<std_srvs::srv::Trigger>("/ivero_slam/start", &start);
  auto stop_service = node->create_service<std_srvs::srv::Trigger>("/ivero_slam/stop", &stop);
  auto cancel_service = node->create_service<std_srvs::srv::Trigger>("/ivero_slam/cancel", &cancel);
  auto status_service = node->create_service<iveroslam_ros2::srv::SystemStatus>("/ivero_slam/get_status", &get_status);

  // publishers
  auto tracked_image_pub = node->create_publisher<sensor_msgs::msg::Image>("/ivero_slam/tracked_image", 10);
  auto output_path_pub = node->create_publisher<std_msgs::msg::String>("/ivero_slam/output_path", 10);
  auto status_pub = node->create_publisher<iveroslam_ros2::msg::SystemStatus>("/ivero_slam/system_status", 10);
  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ivero_slam/map_points", 10);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/ivero_slam/pose", 10);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("/ivero_slam/trajectory", 10);
  auto rgb_pub = node->create_publisher<sensor_msgs::msg::Image>("/ivero_slam/rgb_image", 10);
  auto depth_pub = node->create_publisher<sensor_msgs::msg::Image>("/ivero_slam/depth_image", 10);

  // sigint handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_loop_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  b_continue_session = true;

  // look for realsense device
  rs2::device selected_device;
  try {
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    while (devices.size() == 0 && b_continue_session) {
      RCLCPP_WARN_ONCE(node->get_logger(), "No device connected, please connect a RealSense device.");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      devices = ctx.query_devices();
    }
    if (!b_continue_session) { return -1; }
    selected_device = devices[0];
  } catch (const rs2::error& e) {
    RCLCPP_ERROR(node->get_logger(), "RS2 Error: %s. \n\nIs another process using the realsense device?", e.what());
    return 1;
  }

  // assert camera type
  const auto camera_name = std::string(selected_device.get_info(RS2_CAMERA_INFO_NAME));
  assert(camera_name == "Intel RealSense D455" && "Device must be an Intel RealSense D455.");

  RCLCPP_INFO(node->get_logger(), "\033[1;32mPerforming initial setup.\033[0m");

  // set sensor options
  RCLCPP_INFO(node->get_logger(), "Setting sensor options.");
  std::string error;
  set_sensor_options(selected_device.query_sensors(), error);
  if (!error.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Error setting camera options: %s", error.c_str());
    return 2;
  }

  // Create a configuration for configuring the pipeline with a non default profile
  RCLCPP_INFO(node->get_logger(), "Enabling realsense streams.");
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  RCLCPP_INFO(node->get_logger(), "Starting data capture.");
  rs2::pipeline pipe;
  rs2::pipeline_profile pipe_profile = pipe.start(cfg, CallbackHarness::Callback);

  // Get respective streams for automatic config file creation
  rs2::stream_profile cam_color = pipe_profile.get_stream(RS2_STREAM_COLOR);

  // get extrinsics and intrinsics for rgb camera
  Eigen::Matrix4f T_rgb_camleft = Eigen::Matrix4f::Identity();

  // get config file, create if it doesnt exist
  RCLCPP_INFO(node->get_logger(), "Creating configuration file.");
  const auto serial_number = std::string(selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  const auto config_file = get_config_path(serial_number);
  if (!boost::filesystem::exists(config_file)) { create_config_file(cam_color, config_file); }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  RCLCPP_INFO(node->get_logger(), "Initializing SLAM system.");
  ORB_SLAM3::System SLAM(get_vocab_path(), config_file, ORB_SLAM3::System::RGBD, false, 0, get_date_string());
  float imageScale = SLAM.GetImageScale();

  // variables for orb slam
  double timestamp;
  cv::Mat imDepth, imColor;

  // setup callback variables
  CallbackHarness::timestamp_image = -1.0;
  CallbackHarness::image_ready = false;
  CallbackHarness::count_im_buffer = 0;
  CallbackHarness::offset = 0;
  rs2_intrinsics intrinsics = cam_color.as<rs2::video_stream_profile>().get_intrinsics();
  CallbackHarness::width_img = intrinsics.width;
  CallbackHarness::height_img = intrinsics.height;
  CallbackHarness::output_rgbd = false;

  double t_resize = 0.f;
  double t_track = 0.f;

  bool slam_reset = true;

  std::set<double> lost_timestamps;

  std::string output_path;
  std::string traj_file_name;
  std::string lost_timestamps_file;
  std::string extrinsic_file;

  double previous_state_update = 0.0;
  double state_update_period = 3.0;

  RCLCPP_INFO(node->get_logger(), "Beginning program loop.");
  while (b_continue_session) {
    rclcpp::spin_some(node);

    // access current system state safely
    mtx.lock();
    const auto state = current_state;
    mtx.unlock();

    const auto now = node->now().seconds();
    if (now - previous_state_update > state_update_period) {
      const auto state_str =
          current_state == State::Stopped ? "\033[1;31m Stopped\033[0m" : "\033[1;32m Started\033[0m";
      RCLCPP_INFO(node->get_logger(), "Current system state: %s", state_str);
      previous_state_update = now;
    }

    if (state == State::Stopped) {
      if (!slam_reset) {
        CallbackHarness::output_rgbd = false;
        // publish output folder path
        if (output_results) {
          // output trajectory file and lost timestamps file
          RCLCPP_INFO(node->get_logger(), "Outputting results and compressing... this can take a while.");
          SLAM.SaveTrajectory(traj_file_name, T_rgb_camleft, session_start_time);
          output_timestamps_to_file(lost_timestamps, lost_timestamps_file);
          const auto output_folder_zip = CallbackHarness::output_path + ".zip";
          // try to compress folder
          try {
            zipper::Zipper zipper(output_folder_zip);
            zipper.add(CallbackHarness::output_path, zipper::Zipper::zipFlags::Faster);
            zipper.close();
          } catch (std::runtime_error& re) {
            RCLCPP_ERROR(node->get_logger(), "Error compressing file: %s", output_folder_zip.c_str());
          }
          // publish path to zip file
          std_msgs::msg::String output_path_message;
          output_path_message.data = output_folder_zip;
          output_path_pub->publish(output_path_message);
          // we wait here to ensure data is compressed
          boost::progress_display progress(5);
          for (size_t i = 0; i < 5; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            ++progress;
          }
        }

        // delete directory
        RCLCPP_WARN(node->get_logger(), "Deleting folder: %s", CallbackHarness::output_path.c_str());
        boost::system::error_code ec;
        boost::filesystem::remove_all(CallbackHarness::output_path, ec);
        if (ec) { std::cerr << "Error deleting directory: " << ec.message() << std::endl; }
        slam_reset = true;
      }

      // clear data being filled by the realsense callback
      CallbackHarness::count_im_buffer = 0;
      CallbackHarness::image_ready = false;
      // sleep for a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

    } else if (state == State::Started) {
      if (slam_reset) {
        CallbackHarness::output_rgbd = true;
        session_start_time = now;
        slam_reset = false;
        output_path = setup_output_folders(get_date_string(), config_file);
        traj_file_name = output_path + "/trajectory.txt";
        CallbackHarness::output_path = output_path;
        lost_timestamps_file = output_path + "/lost_timestamps.txt";
        extrinsic_file = output_path + "/T_rgb_camleft.txt";
        lost_timestamps.clear();
      }

      {
        std::unique_lock<std::mutex> lk(CallbackHarness::mutex);
        if (!CallbackHarness::image_ready) CallbackHarness::cond_image_rec.wait(lk);

        if (CallbackHarness::count_im_buffer > 1) cout << CallbackHarness::count_im_buffer - 1 << " dropped frs\n";
        CallbackHarness::count_im_buffer = 0;

        // Copy the data
        timestamp = CallbackHarness::timestamp_image;
        imDepth = CallbackHarness::imDepthCV.clone();
        imColor = CallbackHarness::imColorCV.clone();
        CallbackHarness::image_ready = false;
      }

      if (imageScale != 1.f) {
        int width = imColor.cols * imageScale;
        int height = imColor.rows * imageScale;
        cv::resize(imDepth, imDepth, cv::Size(width, height));
        cv::resize(imColor, imColor, cv::Size(width, height));
      }

      // create message header
      std_msgs::msg::Header header;
      header.stamp = double_to_rostime(timestamp);
      header.frame_id = "left_ir";

      // Stereo images are already rectified.
      const auto Tcw = SLAM.TrackRGBD(imColor, imDepth, timestamp);

      /*
       * Everything below this is to publish over the network, this can and will change frequently
       */
      // publish system status
      current_status.header = header;
      current_status.tracking_status = SLAM.GetTrackingState();
      current_status.is_lost = SLAM.isLost();
      current_status.map_changed = SLAM.MapChanged();
      current_status.has_reset = SLAM.HasReset();
      status_pub->publish(current_status);
      if (current_status.map_changed) {
        // todo: publish trajectory if map changed (make function in system to get full trajectory)
      }
      if (current_status.tracking_status != 2) { lost_timestamps.insert(timestamp); }

      // publish rgb
      auto rgb_msg = cv_bridge::CvImage(header, "rgb8", imColor).toImageMsg();
      rgb_pub->publish(*rgb_msg.get());

      // publish depth
      auto depth_msg = cv_bridge::CvImage(header, "mono16", imDepth).toImageMsg();
      depth_pub->publish(*depth_msg.get());

      // draw keypoints onto rgb image
      cv::Mat tracked_color = imColor.clone();
      cv::Scalar mapped_color(0, 255, 0);
      auto keypoints = SLAM.GetTrackedKeyPointsUn();
      for (const auto& kp : keypoints) {
        cv::Point tl(kp.pt.x - 3, kp.pt.y - 3);
        cv::Point br(kp.pt.x + 3, kp.pt.y + 3);
        cv::Point m(kp.pt.x, kp.pt.y);
        cv::circle(tracked_color, m, 0, mapped_color, 1);
        cv::rectangle(tracked_color, tl, br, mapped_color, 1);
      }

      // publish tracked image
      // cv::Mat tracked_frame = SLAM.GetDrawnFrame();
      // cv::Rect myROI(0, 0, 640, 480);
      // cv::Mat cropImage = tracked_frame(myROI);
      // sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", tracked_frame(myROI)).toImageMsg();
      auto msg = cv_bridge::CvImage(header, "rgb8", tracked_color).toImageMsg();
      tracked_image_pub->publish(*msg.get());

      // publish point cloud
      const auto all_points = SLAM.GetFullPointCloud();
      const auto pointcloud_msg = points_to_cloud(all_points, timestamp);
      cloud_pub->publish(pointcloud_msg);

      // publish pose
      const auto pose_msg = sophus_to_pose_stamped(Tcw, timestamp);
      pose_pub->publish(pose_msg);
    }
  }

  pipe.stop();
  // delete current output folder if ctrl-c
  RCLCPP_WARN(node->get_logger(), "Deleting folder: %s", CallbackHarness::output_path.c_str());
  boost::system::error_code ec;
  boost::filesystem::remove_all(CallbackHarness::output_path, ec);
  if (ec) { std::cerr << "Error deleting directory: " << ec.message() << std::endl; }
}
