#include "aruco_processor/aruco_position.h"

#define RESET_TIME 2

clock_t ArucoPosition::last_creation_time = clock();

ArucoPosition::ArucoPosition() {}

ArucoPosition::ArucoPosition(Mat RTMatrixs) {
  creation_time = clock();
  time_since_last_positon = creation_time - last_creation_time;
  last_creation_time = creation_time;

  RTMatrix = RTMatrixs(Range(0, 3), Range(0, 4));
  tvecs = RTMatrixs(Range(0, 3), Range(3, 4));
  rotMat = RTMatrixs(Range(0, 3), Range(0, 3));

  x = tvecs.at<float>(0);
  y = tvecs.at<float>(1);
  z = tvecs.at<float>(2);

  Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;

  decomposeProjectionMatrix(RTMatrix, cameraMatrix, rotMatrix, transVect,
                            rotMatrixX, rotMatrixY, rotMatrixZ, eulersAngles);

  Mat worldPos(-rotMat.inv() * tvecs);
  float* _wp = worldPos.ptr<float>();
  w_x = _wp[0];
  w_y = _wp[1];
  w_z = _wp[2];

  double* _eA = eulersAngles.ptr<double>();
  ele = _eA[0];
  azi = _eA[1];
  tilt = _eA[2];
}

string ArucoPosition::get_info_string() {
  stringstream output;
  output << std::fixed;
  output << std::setprecision(5);

  output << x << ',' << y << ',' << z << ',';
  output << ele << ',' << azi << ',' << tilt << ',';
  output << w_x << ',' << w_y << ',' << w_z << endl;
  return output.str();
}

string ArucoPosition::get_basic_string() {
  stringstream output;
  output << std::fixed;
  output << std::setprecision(5);

  output << x << ',' << y << ',' << z << ',' << azi << endl;
  return output.str();
}

float ArucoPosition::angle_in_frame() { return atan(x / z) * 180 / 3.14; }
