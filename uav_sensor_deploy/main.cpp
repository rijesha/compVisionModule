#include <aruco_processor/aruco_position.h>
#include <aruco_processor/aruco_processor.h>
#include <common/configuration.h>
#include <multithreaded_interface.h>
#include <position_control_helper.h>
#include <common/cvm_argument_parser.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include "camera_realsense.h"
#include "position_controller.h"
#include "states.hpp"

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // Video write

constexpr float target_width{0.119};
constexpr float default_speed_up{0.20};  // in m/s. (10cm/s)

std::chrono::time_point<std::chrono::high_resolution_clock> start1 =
    std::chrono::high_resolution_clock::now();

clock_t lastDetectedTime;
clock_t startedDataAcquisition;

ofstream logFile;
VideoWriter outputVideo;

class DataHandler {
 private:
  ArUcoProcessor &aruco_processor_;
  std::mutex new_position_available_mutex_;
  std::condition_variable new_position_available_;
  ArucoPosition current_position_;
  int i_{0};

 public:
  DataHandler(ArUcoProcessor &aruco_processor)
      : aruco_processor_(aruco_processor){};

  std::optional<ArucoPosition> wait_for_new_position(float time) {
    std::unique_lock<mutex> lock(new_position_available_mutex_);
    if (new_position_available_.wait_for(lock, time * 1s,
                                         []() { return true; })) {
      return current_position_;
    } else {
      return {};
    }
  }

  ArucoPosition get_current_position() { return current_position_; }

  void process_realsense_data(const RealsenseData &data) {
    if (data.frame1_updated) {
      std::vector<uint8_t> data_vector(
          data.frame1.raw_ptr,
          data.frame1.raw_ptr + data.frame1.height * data.frame1.width);

      Mat image(Size(data.frame1.width, data.frame1.height), CV_8UC1,
                data_vector.data());
      auto optional_position = aruco_processor_.process_raw_frame(image, 17);
      if (optional_position.has_value()) {
        current_position_ = optional_position.value();
        new_position_available_.notify_all();
        cv::putText(image, current_position_.get_uav_string(),
                    cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1,
                    cv::Scalar(255, 255, 255), 2, false);
      }
      outputVideo << image;
    }
  }
};

class MavlinkHandler {
 private:
  bool gains_valid{};
  bool desired_target_valid{};
  mavlink_set_target_ned_t desired_ned;
  mavlink_set_control_gains_t desired_gains;

 public:
  MavlinkHandler(){};

  bool is_valid() { return gains_valid & desired_target_valid; };

  mavlink_set_control_gains_t get_control_gains() { return desired_gains; }

  Vector3f get_desired_position() {
    return {desired_ned.desired_north, desired_ned.desired_east,
            desired_ned.desired_down};
  };

  float get_desired_angle_in_frame() {
    return desired_ned.desired_angle_in_frame;
  };

  void process_mavlink_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_SET_CONTROL_GAINS: {
        mavlink_msg_set_control_gains_decode(&msg, &desired_gains);
        gains_valid = true;
        break;
      }
      case MAVLINK_MSG_ID_SET_TARGET_NED: {
        mavlink_msg_set_target_ned_decode(&msg, &desired_ned);
        desired_target_valid = true;
        break;
      }
      default:
        break;
    }
  }
};

MavlinkHandler mav_handler{};
thread feedback_th;

int main(int argc, const char **argv) {
  cout << std::fixed << std::showpoint;
  cout << std::setprecision(3);

  CVMArgumentParser argparse(argc, argv, true, false, false, false);
  logFile.open("logfile.csv");

  outputVideo.open("out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                   Size(848, 800), false);

  aruco::CameraParameters cam_param;
  cam_param.readFromXMLFile(argparse.calib_file_path);

  ArUcoProcessor aruco_processor(cam_param, target_width);
  PositionController pc{3, 0.5};
  PositionControllerState state_;
  Vector3f desired_angles;
  float desired_yaw_rate;

  DataHandler handler{aruco_processor};

  SerialPort pixhawk_serial("/dev/ttyTHS1", 115200);
  UdpDevice mavproxy_udp("127.0.0.1", 14560);

  MultithreadedInterface pixhawk_interface(pixhawk_serial);
  MultithreadedInterface mavproxy_interface(mavproxy_udp);

  pixhawk_interface.bind_new_msg_callback([&](const mavlink_message_t &msg) {
    mavproxy_interface.write_message(msg);
  });
  mavproxy_interface.bind_new_msg_callback([&](const mavlink_message_t &msg) {
    pixhawk_interface.write_message(msg);
  });

  CameraRealsense camera;
  camera.bind_data_callback(
      [&](const RealsenseData &data) { handler.process_realsense_data(data); });

  feedback_th = thread([&pc, &mavproxy_interface, &handler, &state_,
                        &desired_angles, &desired_yaw_rate]() {
    while (true) {
      std::this_thread::sleep_for(500ms);
      mavlink_message_t msg;

      mavlink_control_gains_t gains;
      gains.pos_pgain = pc.get_pos_pgain();
      gains.pos_pgain_z = pc.get_pos_z_pgain();
      gains.vel_pgain = pc.get_vel_pgain();
      gains.vel_igain = pc.get_vel_igain();
      gains.vel_pgain_z = pc.get_vel_z_pgain();
      gains.vel_igain_z = pc.get_vel_z_igain();
      gains.yaw_pgain = pc.get_yaw_gain();
      mavlink_msg_control_gains_encode(0, 0, &msg, &gains);
      mavproxy_interface.write_message(msg);

      mavlink_target_ned_t target_ned;
      auto current_pos = handler.get_current_position();
      target_ned.north = current_pos.target_ned_vector.x;
      target_ned.east = current_pos.target_ned_vector.y;
      target_ned.down = current_pos.target_ned_vector.z;
      target_ned.yaw = current_pos.target_ned_vector.yaw;
      target_ned.angle_in_frame = current_pos.get_yaw_from_target_centre();

      auto desired_pos = mav_handler.get_desired_position();
      target_ned.desired_north = desired_pos.x;
      target_ned.desired_east = desired_pos.y;
      target_ned.desired_down = desired_pos.z;
      target_ned.desired_angle_in_frame =
          mav_handler.get_desired_angle_in_frame();

      mavlink_msg_target_ned_encode(0, 0, &msg, &target_ned);
      mavproxy_interface.write_message(msg);

      mavlink_control_targets_t control_targets;
      control_targets.current_vel_n = state_.velocity.x;
      control_targets.current_vel_e = state_.velocity.y;
      control_targets.current_vel_d = state_.velocity.z;
      control_targets.vel_n = state_.velocity_desired.x;
      control_targets.vel_e = state_.velocity_desired.y;
      control_targets.vel_d = state_.velocity_desired.z;
      control_targets.acc_n = state_.acceleration_desired.x;
      control_targets.acc_e = state_.acceleration_desired.y;
      control_targets.acc_d = state_.acceleration_desired.z;
      control_targets.roll = desired_angles.y;
      control_targets.pitch = desired_angles.x;
      mavlink_msg_control_targets_encode(0, 0, &msg, &control_targets);
      mavproxy_interface.write_message(msg);
    }
  });

  while (true) {
    auto result = handler.wait_for_new_position(.5);
    if (result.has_value()) {
      auto gains = mav_handler.get_control_gains();
      pc.set_pos_pgain(gains.pos_pgain);
      pc.set_pos_z_pgain(gains.pos_pgain_z);
      pc.set_vel_pgain(gains.vel_pgain);
      pc.set_vel_igain(gains.vel_igain);
      pc.set_vel_z_pgain(gains.vel_pgain_z);
      pc.set_vel_z_igain(gains.vel_igain_z);
      pc.set_yaw_gain(gains.yaw_pgain);

      state_ = pc.run_loop(result.value().target_ned_vector,
                           mav_handler.get_desired_position());

      // cout << result.value().get_uav_string();
      desired_angles = pc.acceleration_to_attitude(
          state_.acceleration_desired.x, state_.acceleration_desired.y,
          result.value().target_ned_vector.yaw);

      desired_yaw_rate =
          pc.get_yaw_rate(result.value().get_yaw_from_target_centre(),
                          mav_handler.get_desired_angle_in_frame());

      cout << "p_: " << desired_angles.y << "r_: " << desired_angles.x
           << "y_: " << desired_yaw_rate << endl;

      if (mav_handler.is_valid()) {
        send_set_attitude_target(pixhawk_interface, desired_angles.y,
                                 desired_angles.x, 0,
                                 -state_.velocity_desired.z / default_speed_up,
                                 desired_yaw_rate, true);
      }
    } else {
      send_set_attitude_target(pixhawk_interface, 0, 0, 0, 0, 0, true);
      enable_offboard_control(pixhawk_interface, false);
    }
  }

  pixhawk_interface.shutdown();
  mavproxy_interface.shutdown();

  logFile.close();
  return -1;
}
