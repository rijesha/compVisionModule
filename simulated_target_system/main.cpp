

#include <multithreaded_interface.h>
#include <stdio.h>  // Standard input/output definitions
#include "position_controller.h"
#include "vector3.h"

SerialPort serial_port("/dev/tnt3", 57600);
MultithreadedInterface mti(serial_port);

// Inputs/Simulated inputs from the vision system
float simulated_bearing_to_centre_rad = 0;
Vector3f target_tv = {0, 0, 0};
Vector3f target_rv = {0, 0, 0};
Vector3f target_wp = {0, 0, 0};

// Converted vision system to usable uav coordinates
PositionAndYawVector target_ned_vector;

float convert_target_tvec_to_angle(Vector3f t_vec) {
  return -atan2f(t_vec.x, t_vec.y) * (180.0f / M_PI);
}

void convert_target_vectors_to_ned_centred_on_target(Vector3f t_vec,
                                                     Vector3f wp_vec) {
  target_ned_vector.x = -wp_vec.z;
  target_ned_vector.y = wp_vec.x;
  target_ned_vector.z = wp_vec.y;
  target_ned_vector.yaw = convert_target_tvec_to_angle(t_vec);
}

void got_new_target_data() {
  convert_target_vectors_to_ned_centred_on_target(target_tv, target_wp);
}

void update_simulated_target_wp_from_uav_ned(float n, float e, float d) {
  target_wp.x = e;
  target_wp.y = d;
  target_wp.z = -n;
}

void update_simulated_target_tv_from_uav_yaw(float yaw_rad) {
  target_rv.y = -yaw_rad * (180.0f / M_PI);
  simulated_bearing_to_centre_rad = -atan2(target_wp.x, target_wp.z);

  float max =
      std::pow(std::pow(target_wp.x, 2) + std::pow(target_wp.z, 2), 0.5);
  target_tv.x = sin(simulated_bearing_to_centre_rad - yaw_rad) * max;
  target_tv.y = cos(simulated_bearing_to_centre_rad - yaw_rad) * max;

  got_new_target_data();
}

void new_msg_callback(mavlink_message_t message) {
  if (message.msgid == 0) {
    cout << "Received HB" << endl;
    mavlink_heartbeat_t heartbeat_msg;
    mavlink_msg_heartbeat_decode(&message, &heartbeat_msg);
    // printf("[compid sysid] = [%d %d] \n", message.compid, message.sysid);
  } else if (message.msgid == 32) {
    // cout << "Got Local Position Ned" << endl;
    mavlink_local_position_ned_t local_position_ned_msg;
    mavlink_msg_local_position_ned_decode(&message, &local_position_ned_msg);
    update_simulated_target_wp_from_uav_ned(local_position_ned_msg.x,
                                            local_position_ned_msg.y,
                                            local_position_ned_msg.z);
  } else if (message.msgid == 30) {
    // cout << "Got Attitude msg" << endl;
    mavlink_attitude_t attitude_msg;
    mavlink_msg_attitude_decode(&message, &attitude_msg);
    update_simulated_target_tv_from_uav_yaw(attitude_msg.yaw);
  }
}

void decode_last_attitude_msg(MultithreadedInterface *mti) {
  auto search = mti->last_messages.find(MAVLINK_MSG_ID_ATTITUDE);
  if (search != mti->last_messages.end()) {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(&(search->second), &att);
    printf("[Roll Pitch Yaw] = [%f %f %f] \n", att.roll, att.pitch, att.yaw);
  } else {
    std::cout << "Attitude Message Not found\n";
  }
}

float get_rate(float current_value, float desired_value) {
  return (desired_value - current_value) * .8;
}

float wrap_angle(float angle) {
  if (angle > 180) {
    return angle - 360;
  }
  return angle;
}

float get_yaw_rate(float current_yaw, float desired_yaw) {
  current_yaw = wrap_angle(current_yaw);
  desired_yaw = wrap_angle(desired_yaw);
  float temp = get_rate(current_yaw, desired_yaw);
  return temp;
}

thread input_th;

float n = -10;
float e = 0;
float d = 0;
float yaw = 0;

void input_thread() {
  int i = 0;
  while (true) {
    switch (i) {
      case 0:
        cout << "input n?" << endl;
        break;
      case 1:
        cout << "input e?" << endl;
        break;
      case 2:
        cout << "input d?" << endl;
        break;
      case 3:
        cout << "input yaw?" << endl;
        break;

      default:
        break;
    }
    float number;
    std::cin >> number;

    switch (i) {
      case 0:
        i = 1;
        n = number;
        break;
      case 1:
        i = 2;
        e = number;
        break;
      case 2:
        i = 3;
        d = number;
        break;
      case 3:
        i = 0;
        yaw = number;
        break;

      default:
        break;
    }
  }
}

PositionController ac;

void mavlinkMessageCallback(mavlink_message_t *msg) {
  printf("Got msg %d!\n", msg->msgid);
  /*
  if (msg->msgid == MAVLINK_MSG_ID_Set_Gains)
  {
      mavlink_set_gains_t msg;
  }
   */
}

int main() {
  // MavlinkServer server;
  // server.bindMessageSubscriber(mavlinkMessageCallback);
  // cout << "Mavlink Server Started" << endl;

  int i = 0;
  while (true) {
    i++;
    mavlink_message_t message;

    mavlink_heartbeat_t heartbeat_msg;
    heartbeat_msg.base_mode = i;
    mavlink_msg_heartbeat_encode(0, 0, &message, &heartbeat_msg);
    // server.sendMessage(&message);
    this_thread::sleep_for(chrono::milliseconds(1000));
  }

  chrono::milliseconds k1(5000);
  // p.enable_attitude_messages(100000);
  // p.enable_local_position_estimate_messages(100000);
  // p.toggle_offboard_control(true);

  mti.bind_new_msg_callback(new_msg_callback);
  cout << "Bound Message CallBack" << endl;

  input_th = thread([=]() { input_thread(); });

  int count = 0;
  while (true) {
    count++;
    if (count == 10) {
      printf("desired pos n: %f, e: %f, d: %f, with yaw: %f\n", n, e, d, yaw);
      printf("current pos n: %f, e: %f, d: %f, with yaw: %f\n",
             target_ned_vector.x, target_ned_vector.y, target_ned_vector.z,
             target_ned_vector.yaw);
      printf("right acc %f, forward acc %f, rot %f \n", ac.right_acc,
             ac.forward_acc, ac.rot * 180 / 3.14);
      printf("rot right acc %f, rot forward acc %f \n", ac.rot_right_acc,
             ac.rot_forward_acc);
      printf("pitch target %f, roll target %f \n", ac.pitch_target,
             ac.roll_target);

      count = 0;
    }
    ac.run_loop(target_ned_vector, {n, e, d});
    ac.acceleration_to_attitude(ac.acc_desi.x, ac.acc_desi.y, -target_rv.y);

    // Vector3f pos_err = Vector3f{n, e, d} - target_ned_vector;
    // Vector3f pos_pgain = {0.05, 0.050, 0.050};

    ////Calculating desired velocity
    // Vector3f vel_desi = pos_err.multiplyGain(pos_pgain);

    // ac.acceleration_to_attitude(vel_desi.x, vel_desi.y, -target_rv.y);

    // float rel_vert_vel = (ac->get_desired_velocity().y / DEFAULT_SPEED_UP);

    // p.send_set_attitude_target(0, 0, 0, 0, 0, true); // dont move  //
    // p.send_set_attitude_target(ac.pitch_target, ac.roll_target, 0.0, .2,
    // get_yaw_rate(target_ned_vector.yaw, yaw), true); // gain altitude while
    // pointing north

    // p.send_set_attitude_target(0, 0, 0, 0, 0, true);
    this_thread::sleep_for(chrono::milliseconds(100));
  }

  // p.toggle_offboard_control(false);
  // p.shutdown();
  return 0;
}
