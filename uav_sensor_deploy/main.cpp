#include <aruco_processor/aruco_position.h>
#include <aruco_processor/aruco_processor.h>
#include <common/configuration.h>
#include <multithreaded_interface.h>
#include <position_control_helper.h>
#include <common/cvm_argument_parser.hpp>
#include <common/navigational_state.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include "camera_realsense.h"
#include "position_controller.h"
#include "states.hpp"

constexpr float target_width{0.158};
constexpr float default_speed_up{0.20};  // in m/s. (10cm/s)

NavigationalState<State> *ap = new AutoPilotState();
NavigationalState<State> *na = new NormalApproach();
NavigationalState<State> *da = new DataAcquisitionState();
NavigationalState<State> *po = new PullOutState();

NavigationalState<State> *xt = new CrossTest();
NavigationalState<State> *ct = new CircleTest();
// NavigationalState<State> *ph = new PositionHold();

std::chrono::time_point<std::chrono::high_resolution_clock> start1 =
    std::chrono::high_resolution_clock::now();

clock_t lastDetectedTime;
clock_t startedDataAcquisition;

ofstream logFile;
int i = 0;

class DataHandler {
 private:
  ArUcoProcessor &aruco_processor_;
  std::mutex new_position_available_mutex_;
  std::condition_variable new_position_available_;
  ArucoPosition current_position_;

 public:
  DataHandler(ArUcoProcessor &aruco_processor)
      : aruco_processor_(aruco_processor){};

  std::optional<ArucoPosition> wait_for_new_position(float time) {
    std::unique_lock<mutex> lock(new_position_available_mutex_);
    if (new_position_available_.wait_for(lock, time * 1s,
                                         [] { return i == 1; })) {
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
      auto optional_position = aruco_processor_.process_raw_frame(image);
      if (optional_position.has_value()) {
        current_position_ = optional_position.value();
        new_position_available_.notify_all();
      }

      if ((std::chrono::high_resolution_clock::now() - start1).count() >
          10000000000) {
        printf("10s has happened %d %ld \n", i,
               (std::chrono::high_resolution_clock::now() - start1).count());
        start1 = std::chrono::high_resolution_clock::now();
        i = 0;
      }

      // imwrite(std::to_string(i) + "test.png", image);
    }
    if (data.frame2_updated) {
      std::vector<uint8_t> data_vector(
          data.frame2.raw_ptr,
          data.frame2.raw_ptr + data.frame2.height * data.frame2.width);

      Mat image(Size(data.frame2.width, data.frame2.height), CV_8UC1,
                data_vector.data());
      // imwrite(std::to_string(i) + "test_2.png", image);
    }
    i++;
  }
};

int main(int argc, const char **argv) {
  cout << std::fixed << std::showpoint;
  cout << std::setprecision(3);

  CVMArgumentParser argparse(argc, argv, true, false, false, false);
  logFile.open("logfile.csv");

  aruco::CameraParameters cam_param;
  cam_param.readFromXMLFile(argparse.calib_file_path);

  // NavigationalState<State> *ns = ap;

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

      auto desired_angles = pc.acceleration_to_attitude(
          state.acceleration_desired.x, state.acceleration_desired.y,
          result.value().target_ned_vector.yaw);

      auto desired_yaw_rate =
          pc.get_yaw_rate(result.value().get_yaw_from_target_centre(), 0);

      send_set_attitude_target(pixhawk_interface, desired_angles.y, desired_angles.x, 0,
                               -state.velocity_desired.z / default_speed_up,
                               desired_yaw_rate, true);

    } else {
      enable_offboard_control(pixhawk_interface, false);
      // do something else to kick out of autopilot mode
    }
  }

  int count = 0;
  State lastState = AP;
  State currentState = AP;

  bool firstDA = true;
  bool firstPostDA = false;
  float lastyaw = 0;
  auto start = std::chrono::high_resolution_clock::now();
  double total_time = 0;
  int downsample = 0;

  // while (true) {
  // sleep(10);
  // startimageCaputre = clock();
  // current_position = lp.processImage();

  // if (argparse.saveVideo){
  //    imwrite("captured/" + to_string(count) + ".jpg", lp.original);
  //    imwrite("captured/" + to_string(count) + "marked.jpg",
  //    lp.arProc.drawMarkersAndAxis(lp.original));
  //}

  /*
  if (!current_position.emptyPosition)
  {
      lastDetectedTime = clock();
      //cout << "current position azi " << -current_position.azi << endl;
      lastyaw = pc->getLastAttitudeYaw();

      if (!argparse.quiet)
      {
          cout << "angle of uav " << lastyaw * 180 / 3.14 << endl;
          cout << current_position.w_z << " " << current_position.w_x <<
  endl; cout << "world_z: " << -current_position.w_y << " z in frame: " <<
  -current_position.y << endl;
      }

      //pc->update_current_position(x_new, y_new, -current_position.w_y,
  lastyaw, startimageCaputre);
      //cout << current_position.w_x << " " << current_position.w_z << " "
  << -current_position.w_y << endl;
  }
  */

  // ns = ns->returnNextState(current_position);
  // currentState = ns->currentState();

  /*
  downsample++;
  if (currentState == DA)
  {
      firstPostDA = true;
      if (downsample == 10){
          serial_port._write_port(&enable_magnet, 1);
          downsample = 0;
      }

      if (firstDA)
      {
          vibration_file << "NEW VIBRATION" << endl
                         << "NEW VIBRATION" << endl
                         << "NEW VIBRATION" << endl;
          firstDA = false;
      }
  }
  else
  {
      if (firstPostDA){
          serial_port._write_port(&release_magnet, 1);
          firstPostDA = false;
          downsample = 10;
      }
      if (downsample == 10){
          serial_port._write_port(&release_magnet, 1);
          downsample = 0;
      }

  }
  */

  /*
  if (currentState == AP && lastState != AP)
      pc->toggle_offboard_control(false);
  else if (lastState == AP && currentState != AP)
      pc->toggle_offboard_control(true);
  */

  /*
  desired_position = ns->computeDesiredPosition(current_position);

  ac->run_loop({current_position.w_x, current_position.w_y,
  current_position.w_z}, {desired_position.x, desired_position.y,
  desired_position.z});

  ac->acceleration_to_attitude(-ac->acc_desi.z, ac->acc_desi.x,
  -current_position.azi);

  float rel_vert_vel = (ac->get_desired_velocity().y/DEFAULT_SPEED_UP);

  float yaw_rate = -current_position.angle_in_frame() * 0.4;

  if (!ac->reinitialize_state){
      pc->send_set_attitude_target(ac->pitch_target, ac->roll_target,
  ac->yaw_target, rel_vert_vel, yaw_rate, true);
  }

  send_mavlink_debug++;
  if (send_mavlink_debug == 5){
      send_mavlink_debug = 0;
      hil_actuator_controls.controls[0] = current_position.w_x;
      hil_actuator_controls.controls[1] = current_position.w_y;
      hil_actuator_controls.controls[2] = current_position.w_z;
      hil_actuator_controls.controls[3] = desired_position.x;
      hil_actuator_controls.controls[4] = desired_position.y;
      hil_actuator_controls.controls[5] = desired_position.z;
      hil_actuator_controls.controls[6] = ac->get_desired_velocity().x;
      hil_actuator_controls.controls[7] = ac->get_desired_velocity().y;
      hil_actuator_controls.controls[8] = ac->get_desired_velocity().z;
      hil_actuator_controls.controls[9] = ac->acc_desi.x;
      hil_actuator_controls.controls[10] = ac->acc_desi.y;
      hil_actuator_controls.controls[11] = ac->acc_desi.z;
      hil_actuator_controls.controls[12] = current_position.azi;
      hil_actuator_controls.controls[13] =
  current_position.angle_in_frame();

          mavlink_msg_hil_actuator_controls_encode(0, 0,
  &hil_actuator_controls_message, &hil_actuator_controls);
      pc->mti->write_message(hil_actuator_controls_message);
  }


  //mavlink_message_t attitude_target_message;


  //HIL_ACTUATOR_CONTROLS

  lastState = currentState;
  logFile << time(0) << "," << ac->get_state_string() << endl;
  count++;
*/
  /*
  if (!argparse.quiet)
  {
      //cout << ac->get_state_string() << endl << endl;
      cout << ac->forward_acc << "," << ac->right_acc << ",   " << " ";

      cout << current_position.azi << ",   " << " ";

      cout << ac->rot_forward_acc << "," << ac->rot_right_acc << ",   " << "
  ";

      cout << yaw_rate << "," << rel_vert_vel << endl << endl;
      /*
      cout << currentState << endl;
      cout << "angle in frame " << desired_position.azi << endl;
      cout << "current angle global " << desired << endl;
      cout << "x_current: " << desired_position.z << "y_current: " <<
  desired_position.x << endl;

      //cout << "desired angle " << -current_position.azi +
  desired_position.azi << endl; cout << "desired angle global " << desired
  << endl; cout << "x_desired_before: " << desired_position.z <<
  "y_desired_before: " << desired_position.x << endl; cout <<
  "x_desired_after: " << x_new << "y_desired_after: " << y_new <<
  "z_desired_after" << -desired_position.y << endl;
      //
  }*/
  /*
  if (argparse.saveTiming)
  {
      std::chrono::duration<double> elapsed =
  std::chrono::high_resolution_clock::now() - start;
  total_time += elapsed.count();
  std::cout << "Elapsed time: " << elapsed.count() << " s Averge time: " <<
  total_time / count << "s\n";

      start = std::chrono::high_resolution_clock::now();
  }*/
  //}
  pixhawk_interface.shutdown();
  mavproxy_interface.shutdown();

  logFile.close();
  return -1;
}
