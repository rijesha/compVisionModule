#include "aruco_processor/aruco_position.h"

ArucoPosition::ArucoPosition(Mat RTMatrixs) {
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

  target_ned_vector = {-w_z, w_x, -w_y, azi};
}

float ArucoPosition::get_yaw_from_target_centre() {
  return -atan2f(x, z) * (180.0f / static_cast<float>(M_PI));
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

string ArucoPosition::get_uav_string() {
  stringstream output;
  output << std::fixed;
  output << std::setprecision(4);

  output << "N: " << target_ned_vector.x << "m E: " << target_ned_vector.y
         << "m D: " << target_ned_vector.z
         << "m Yaw: " << target_ned_vector.yaw << endl;
  return output.str();
}
