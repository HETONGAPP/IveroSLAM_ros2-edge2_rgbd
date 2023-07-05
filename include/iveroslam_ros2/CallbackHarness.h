#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <optional>
#include <stdlib.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core/core.hpp>

/// @brief This is a fully static class to encapsulate the callback for the realsense device and how it interacts with
/// orb slam
class CallbackHarness {
public:
  /// @brief Main callback for realsense device it performs multiple functions:
  /// 1. Determine type of frame being read and will update the current measurements for SLAM
  /// 2. Process RGBD data and add to buffer
  /// @param frame to process
  static void Callback(const rs2::frame& frame);

  static std::string output_path;
  /// @brief
  static std::mutex mutex;
  /// @brief
  static std::condition_variable cond_image_rec;
  /// @brief
  static cv::Mat imDepthCV, imColorCV;
  /// @brief
  static int width_img, height_img;
  /// @brief
  static double timestamp_image;
  /// @brief
  static bool image_ready;
  /// @brief
  static int count_im_buffer; // count dropped frames
  /// @brief
  static double offset;
  /// @brief 
  static bool output_rgbd;

private:
  // Disallow creating an instance of this object
  CallbackHarness() {}
};

#include "CallbackHarnessImp.hpp"
