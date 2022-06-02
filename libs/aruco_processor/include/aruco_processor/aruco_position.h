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

class ArucoPosition {
 public:
  ArucoPosition(Mat RTmatrix);
  ArucoPosition(bool valid);

  bool valid = false;
  Mat rvecs, tvecs, RTMatrix;
  Mat eulersAngles;

  Mat rotMat;
  string get_info_string();
  string get_basic_string();
  string get_uav_string();

  float x = 0, y = 0, z = 0;
  float ele = 0, azi = 0, tilt = 0;
  float w_x = 0, w_y = 0, w_z = 0;
  PositionAndYawVector target_ned_vector;

  float get_yaw_from_target_centre();
};
