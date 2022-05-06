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

int main(int argc, const char **argv) {
  cout << std::fixed << std::showpoint;
  cout << std::setprecision(3);

  CVMArgumentParser argparse(argc, argv, true, false, false, false);
  logFile.open("logfile.csv");

  outputVideo.open("out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                   Size(848, 800), false);

  aruco::CameraParameters cam_param;
  cam_param.readFromXMLFile(argparse.calib_file_path);

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

  ArUcoProcessor aruco_processor(cam_param, target_width);
  PositionController pc;

  DataHandler handler{aruco_processor};

  CameraRealsense camera;
  camera.bind_data_callback(
      [&](const RealsenseData &data) { handler.process_realsense_data(data); });

  Vector3f desired_position;
  while (true) {
    auto result = handler.wait_for_new_position(.5);
    if (result.has_value()) {
      auto state =
          pc.run_loop(result.value().target_ned_vector, desired_position);

      cout << result.value().get_uav_string();
      auto desired_angles = pc.acceleration_to_attitude(
          state.acceleration_desired.x, state.acceleration_desired.y,
          result.value().target_ned_vector.yaw);

      auto desired_yaw_rate =
          pc.get_yaw_rate(result.value().get_yaw_from_target_centre(), 0);

      cout << "p_: " << desired_angles.y << "r_: " << desired_angles.x
           << "y_: " << desired_yaw_rate << endl;
      send_set_attitude_target(
          pixhawk_interface, desired_angles.y, desired_angles.x, 0,
          -state.velocity_desired.z / default_speed_up, desired_yaw_rate, true);

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
