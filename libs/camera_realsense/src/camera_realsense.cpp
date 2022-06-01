#include "camera_realsense.h"

CameraRealsense::CameraRealsense() {
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

  auto pipe_profile = pipe.start(cfg);

  auto fisheye_stream_1 = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
  auto fisheye_intrinsics_1 =
      fisheye_stream_1.as<rs2::video_stream_profile>().get_intrinsics();
  auto body_fisheye_extr_1 = fisheye_stream_1.get_extrinsics_to(
      pipe_profile.get_stream(RS2_STREAM_POSE));

  auto fisheye_stream_2 = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 2);
  auto fisheye_intrinsics_2 =
      fisheye_stream_2.as<rs2::video_stream_profile>().get_intrinsics();
  auto body_fisheye_extr_2 = fisheye_stream_2.get_extrinsics_to(
      pipe_profile.get_stream(RS2_STREAM_POSE));

  process_thread = std::thread([=]() { process_function(); });
}

void CameraRealsense::process_function() {
  int i = 0;
  while (!shutdown_) {
    RealsenseData data;
    rs2::frame frame1;
    rs2::frame frame2;

    auto frames = pipe.wait_for_frames();
    auto pose_frame = frames.get_pose_frame();
    auto fisheye_frame_1 = frames.get_fisheye_frame(1);
    auto fisheye_frame_2 = frames.get_fisheye_frame(2);

    data.pose_data = pose_frame.get_pose_data();

    fisheye_frame_1.keep();
    data.frame1.raw_ptr =
        static_cast<const uint8_t *>(fisheye_frame_1.get_data());
    data.frame1.width = fisheye_frame_1.get_width();
    data.frame1.height = fisheye_frame_1.get_height();

    fisheye_frame_2.keep();
    data.frame2.raw_ptr =
        static_cast<const uint8_t *>(fisheye_frame_2.get_data());
    data.frame2.width = fisheye_frame_2.get_width();
    data.frame2.height = fisheye_frame_2.get_height();
    /*
      for (auto &&frame : frames()) {
        if (auto mf = frame.as<rs2::motion_frame>()) {
          if (mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
            data.gyro_data = mf.get_motion_data();
            data.gyro_updated = true;
          }
          if (mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
            data.accel_data = mf.get_motion_data();
            data.accel_updated = true;
          }
        }
      }
      */
    data.frame_index = i++;
    for (auto cb : data_callbacks_) {
      cb(data);
    }
  }
}

void CameraRealsense::bind_data_callback(RealsenseDataCallback cb) {
  data_callbacks_.push_back(cb);
}

void CameraRealsense::shutdown() { shutdown_ = true; }
