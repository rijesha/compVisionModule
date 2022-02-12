#include "camera_realsense.h"

CameraRealsense::CameraRealsense() {
  pipe.start();
  process_thread = std::thread([=]() { process_function(); });
}

void CameraRealsense::process_function() {
  int i = 0;
  while (!shutdown_) {
    RealsensePtrData data;
    for (auto&& frame : pipe.wait_for_frames()) {
      if (auto pf = frame.as<rs2::pose_frame>()) {
        data.pose_data = pf.get_pose_data();
        data.pose_updated = true;
      } else if (auto mf = frame.as<rs2::motion_frame>()) {
        if (mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
          data.gyro_data = mf.get_motion_data();
          data.gyro_updated = true;
        }
        if (mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
          data.accel_data = mf.get_motion_data();
          data.accel_updated = true;
        }
      } else if (auto vf = frame.as<rs2::video_frame>()) {
        if (vf.get_profile().stream_index() == 1) {
          data.frame_data1.data_ptr = vf.get_data();
          data.frame_data1.width = vf.get_width();
          data.frame_data1.height = vf.get_height();
          data.frame_1_updated = true;

        } else if (vf.get_profile().stream_index() == 2) {
          data.frame_data2.data_ptr = vf.get_data();
          data.frame_data2.width = vf.get_width();
          data.frame_data2.height = vf.get_height();
          data.frame_2_updated = true;
        }
      }
    }
    data.frame_index = i++;
    for (auto cb : data_callbacks_) {
      cb(data);
    }
  }
}

void CameraRealsense::bind_data_callback(RealsensePtrDataCallback cb) {
  data_callbacks_.push_back(cb);
}

void CameraRealsense::shutdown() { shutdown_ = true; }
