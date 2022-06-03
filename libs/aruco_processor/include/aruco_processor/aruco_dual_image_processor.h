#pragma once

#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include "aruco.h"
#include "aruco_processor/aruco_position.h"
#include "undistort_image.h"

using namespace std;
using namespace cv;
using namespace aruco;

class ArUcoDualImageProcessor {
 private:
  Ptr<Dictionary> dictionary;
  Ptr<MarkerDetector> detector;
  Ptr<MarkerPoseTracker> pose_tracker;

  CameraParameters cam_params_1_;
  CameraParameters cam_params_2_;
  float target_size_;
  std::optional<ArucoPosition> calculate_pose(Marker& marker);
  std::optional<Marker> get_marker(int marker_id);

 public:
  ArUcoProcessor(CameraParameters cam_params_1, CameraParameters cam_params_2, float targetSize);

  std::optional<std::pair<Marker, ArucoPosition>> process_raw_frame(
      Mat image1, Mat image2, int markerID);
  Mat draw_markers_and_axis(Mat image, Marker marker);
};
