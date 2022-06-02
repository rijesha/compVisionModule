#include <aruco_processor/aruco_position.h>
#include <aruco_processor/aruco_processor.h>
#include <common/configuration.h>
#include <multithreaded_interface.h>
#include <position_control_helper.h>
#include <common/cvm_argument_parser.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <semaphore>
#include <thread>
#include "camera_realsense.h"
#include "position_controller.h"

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // Video write

constexpr float target_width{0.146};
constexpr float default_speed_up{0.20};  // in m/s. (10cm/s)

std::chrono::steady_clock::time_point start_time =
    std::chrono::steady_clock::now();

ofstream logFile;
VideoWriter outputVideo;

bool automatic_mode_active = false;

struct aruco_position_frame {
  ArucoPosition position{false};
  std::chrono::steady_clock::time_point frame_time;
  std::chrono::steady_clock::time_point position_time;
};

class DataHandler {
 private:
  ArUcoProcessor &aruco_processor_;
  std::binary_semaphore signal{0};
  aruco_position_frame current_position_;
  
 public:
  DataHandler(ArUcoProcessor &aruco_processor)
      : aruco_processor_(aruco_processor){};

  std::optional<aruco_position_frame> get_new_position() {
    if (signal.try_acquire()) {
      return current_position_;
    }
    return {};
  }

  aruco_position_frame get_current_position() { return current_position_; }

  void process_realsense_data(const RealsenseData &data) {
    current_position_.frame_time = data.frame_time;

    Mat image(Size(data.frame1.width, data.frame1.height), CV_8UC1,
              (void *)data.frame1.raw_ptr);
    Mat image_2(Size(data.frame2.width, data.frame2.height), CV_8UC1,
                (void *)data.frame2.raw_ptr);
    
    /*count++;
    if ((count % 25) == 0) {
      cout << "saving image" << endl;
      imwrite("cam1_" + to_string(count1) + ".jpg", image);
      imwrite("cam2_" + to_string(count1) + ".jpg", image_2);
      count1++;
    }*/

    auto optional_position = aruco_processor_.process_raw_frame(image, 17);
    current_position_.position_time = std::chrono::steady_clock::now();

    if (optional_position.has_value()) {
      current_position_.position = optional_position.value().second;
      cout << optional_position.value().second.get_uav_string();
      cv::putText(image, optional_position.value().second.get_uav_string(),
                  cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1,
                  cv::Scalar(255, 255, 255), 2, false);
      if (automatic_mode_active) {
        cv::putText(image, "Command Active", cv::Point(50, 100),
                    cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 2,
                    false);
      }
      image = aruco_processor_.draw_markers_and_axis(
          image, optional_position.value().first);
    }

    Mat dst;
    cv::hconcat(image, image_2, dst);

    outputVideo << dst;
    if (optional_position.has_value()) signal.release();
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

  void process_ardupilot_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t heartbeat_msg;
        mavlink_msg_heartbeat_decode(&msg, &heartbeat_msg);
        automatic_mode_active = heartbeat_msg.custom_mode == 20;
        break;
      }
      default:
        break;
    }
  }

  void process_mavlink_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_SET_CONTROL_GAINS: {
        printf("got control gains \n");
        mavlink_msg_set_control_gains_decode(&msg, &desired_gains);
        gains_valid = true;
        if (gains_valid && desired_target_valid) {
          printf("Control Enabled \n");
        }
        break;
      }
      case MAVLINK_MSG_ID_SET_TARGET_NED: {
        printf("got target ned \n");
        mavlink_msg_set_target_ned_decode(&msg, &desired_ned);
        desired_target_valid = true;
        if (gains_valid && desired_target_valid) {
          printf("Control Enabled \n");
        }
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
  logFile << "frame_time, position_time, start_control_time, control_time, "
             "control_active, "
             "raw_north, raw_east, raw_down, "
             "current_north , current_east, "
             "current_down, desired_north , desired_east, desired_down, "
             "vel_north, vel_east, vel_down, desired_vel_north, "
             "desired_vel_east, desired_vel_down, speed_up_ratio, "
             "pitch_target, roll_target, yaw_rate "
          << endl;

  outputVideo.open("out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                   Size(1696, 800), false);

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
    mav_handler.process_mavlink_message(msg);
  });

  CameraRealsense camera;
  camera.bind_data_callback(
      [&](const RealsenseData &data) { handler.process_realsense_data(data); });

  printf("Camera Opened\n");
  feedback_th = thread([&pc, &mavproxy_interface, &handler, &state_,
                        &desired_angles, &desired_yaw_rate]() {
    while (true) {
      std::this_thread::sleep_for(500ms);
      mavlink_message_t msg;
      mavlink_attitude_t att;
      mavlink_msg_attitude_encode(1, 0, &msg, &att);
      mavproxy_interface.write_message(msg);

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
      target_ned.north = current_pos.position.target_ned_vector.x;
      target_ned.east = current_pos.position.target_ned_vector.y;
      target_ned.down = current_pos.position.target_ned_vector.z;
      target_ned.yaw = current_pos.position.target_ned_vector.yaw;
      target_ned.angle_in_frame =
          current_pos.position.get_yaw_from_target_centre();

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

  int count = 0;
  int missed_count = 0;

  std::chrono::steady_clock::time_point start_control_time =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point control_time =
      std::chrono::steady_clock::now();

  while (true) {
    auto result = handler.get_new_position();
    if (result.has_value()) {
      start_control_time = std::chrono::steady_clock::now();
      auto gains = mav_handler.get_control_gains();
      pc.set_pos_pgain(gains.pos_pgain);
      pc.set_pos_z_pgain(gains.pos_pgain_z);
      pc.set_vel_pgain(gains.vel_pgain);
      pc.set_vel_igain(gains.vel_igain);
      pc.set_vel_z_pgain(gains.vel_pgain_z);
      pc.set_vel_z_igain(gains.vel_igain_z);
      pc.set_yaw_gain(gains.yaw_pgain);

      state_ = pc.run_loop(result.value().position.target_ned_vector,
                           mav_handler.get_desired_position());

      // cout << result.value().get_uav_string();
      desired_angles = pc.acceleration_to_attitude(
          state_.acceleration_desired.x, state_.acceleration_desired.y,
          result.value().position.target_ned_vector.yaw);

      desired_yaw_rate =
          pc.get_yaw_rate(result.value().position.get_yaw_from_target_centre(),
                          mav_handler.get_desired_angle_in_frame());

      // cout << "p_: " << desired_angles.y << "r_: " << desired_angles.x
      //    << "y_: " << desired_yaw_rate << endl;
      control_time = std::chrono::steady_clock::now();

      logFile << std::chrono::duration_cast<std::chrono::milliseconds>(
                     result.value().frame_time - start_time)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     result.value().position_time - start_time)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     start_control_time - start_time)
                     .count()
              << ","
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     control_time - start_time)
                     .count()
              << "," << automatic_mode_active << ","
              << result.value().position.target_ned_vector.x << ","
              << result.value().position.target_ned_vector.y << ","
              << result.value().position.target_ned_vector.z << ","
              << state_.position.x << "," << state_.position.y << ","
              << state_.position.z << ","
              << mav_handler.get_desired_position().x << ","
              << mav_handler.get_desired_position().y << ","
              << mav_handler.get_desired_position().z << ","
              << state_.velocity.x << "," << state_.velocity.y << ","
              << state_.velocity.z << "," << state_.velocity_desired.x << ","
              << state_.velocity_desired.y << "," << state_.velocity_desired.z
              << "," << -state_.velocity_desired.z / default_speed_up << ","
              << desired_angles.y << "," << desired_angles.x << ","
              << desired_yaw_rate << endl;

      if (mav_handler.is_valid()) {
        desired_angles.y = 0;
        desired_angles.x = 0;
        send_set_attitude_target(pixhawk_interface, desired_angles.y,
                                 desired_angles.x, 0,
                                 -state_.velocity_desired.z / default_speed_up,
                                 desired_yaw_rate, true);
      }

      count++;
      missed_count = 0;
    } else {
      missed_count++;
      if (missed_count == 250) {
        missed_count = 0;
        send_set_attitude_target(pixhawk_interface, 0, 0, 0, 0, 0, true);
        enable_offboard_control(pixhawk_interface, false);
      }
      std::this_thread::sleep_for(2ms);
    }
  }

  pixhawk_interface.shutdown();
  mavproxy_interface.shutdown();

  logFile.close();
  return -1;
}
