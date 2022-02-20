#pragma once

#include <math.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "aruco.h"
#include "vector3.h"

using namespace std;
using namespace cv;
using namespace aruco;

/*
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
*/
class ArucoPosition {
 private:
  static clock_t last_creation_time;

 public:
  ArucoPosition();
  ArucoPosition(Mat RTmatrix);

  Mat rvecs, tvecs, RTMatrix;
  Mat eulersAngles;

  Mat rotMat;
  string get_info_string();
  string get_basic_string();
  clock_t creation_time;
  clock_t time_since_last_positon;

  float x = 0, y = 0, z = 0;
  float ele = 0, azi = 0, tilt = 0;
  float w_x = 0, w_y = 0, w_z = 0;
  float angle_in_frame();
};
