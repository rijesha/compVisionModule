#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "aruco.h"

using namespace std;
using namespace cv;
using namespace aruco;

class UndistortImage {
 private:
  CameraParameters& cam_params_;
  Mat new_camera_matrix_;

 public:
  UndistortImage(CameraParameters& cam_params);
  void undistort_acquired_image(Mat img, Mat* dstImg);
  Marker undistort_marker_points(Marker src_pts);
};
