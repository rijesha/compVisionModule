#pragma once

#include <iomanip>
#include <iostream>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>
#include <thread>
#include <vector>

struct RealSenseImageData {
  const uint8_t* raw_ptr;
  int width, height;
};

struct RealsenseData {
  int frame_index;
  rs2_vector gyro_data{};
  rs2_vector accel_data{};
  rs2_pose pose_data{};
  RealSenseImageData frame1{};
  RealSenseImageData frame2{};
  bool gyro_updated{}, accel_updated{}, pose_updated{}, frame1_updated{},
      frame2_updated{};
};

typedef std::function<void(const RealsenseData& data)> RealsenseDataCallback;

class CameraRealsense {
 public:
  CameraRealsense();

  void shutdown();
  void bind_data_callback(RealsenseDataCallback);

 private:
  std::vector<RealsenseDataCallback> data_callbacks_;

  rs2::pipeline pipe;

  std::thread process_thread;
  void process_function();
  bool shutdown_{};
};
