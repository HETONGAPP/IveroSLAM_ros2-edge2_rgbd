#include "CallbackHarness.h"
#include "utils.h"

#include <opencv2/opencv.hpp>

std::string CallbackHarness::output_path;
std::mutex CallbackHarness::mutex;
std::condition_variable CallbackHarness::cond_image_rec;

bool CallbackHarness::output_rgbd;
cv::Mat CallbackHarness::imDepthCV, CallbackHarness::imColorCV;
int CallbackHarness::width_img, CallbackHarness::height_img;
double CallbackHarness::timestamp_image;
bool CallbackHarness::image_ready;
int CallbackHarness::count_im_buffer;
double CallbackHarness::offset;
rs2::spatial_filter spat(1.0, 50.0, 5.0, 0);

void filterDepthFrame(rs2::depth_frame& depth_frame) {
  depth_frame = spat.process(depth_frame);
}

void CallbackHarness::Callback(const rs2::frame& frame) {
  static double prev_output_time = 0.0;
  std::unique_lock<std::mutex> lock(mutex);

  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    count_im_buffer++;

    rs2::align align(RS2_STREAM_COLOR);
    // align frameset to color stream
    auto aligned_fs = align.process(fs);
    auto timestamp = fs.get_timestamp() * 1e-3;

    double new_timestamp_image = fs.get_timestamp() * 1e-3;
    if (abs(timestamp_image - new_timestamp_image) < 0.001) {
      // cout << "Two frames with the same timeStamp!!!\n";
      count_im_buffer--;
      return;
    }

    rs2::video_frame rgb_frame = fs.get_color_frame();
    rs2::depth_frame depth_frame = aligned_fs.get_depth_frame();

    imDepthCV = cv::Mat(cv::Size(width_img, height_img), CV_16UC1, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
    imColorCV = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(rgb_frame.get_data()), cv::Mat::AUTO_STEP);

    timestamp_image = fs.get_timestamp() * 1e-3;
    image_ready = true;

    lock.unlock();
    cond_image_rec.notify_all();
  }
}
