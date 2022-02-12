#pragma once

#include <iomanip>
#include <iostream>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>
#include <thread>
#include <vector>

struct RealSenseImageData {
  const void* data_ptr;
  int width, height;
};

struct RealsensePtrData {
  int frame_index;
  rs2_vector gyro_data{};
  rs2_vector accel_data{};
  rs2_pose pose_data{};
  RealSenseImageData frame_data1{};
  RealSenseImageData frame_data2{};
  bool gyro_updated, accel_updated, pose_updated, frame_1_updated,
      frame_2_updated;
};

typedef std::function<void(const RealsensePtrData& data)>
    RealsensePtrDataCallback;

class CameraRealsense {
 public:
  CameraRealsense();

  void shutdown();
  void bind_data_callback(RealsensePtrDataCallback);

 private:
  std::vector<RealsensePtrDataCallback> data_callbacks_;

  rs2::pipeline pipe;

  std::thread process_thread;
  void process_function();
  bool shutdown_{};
};
