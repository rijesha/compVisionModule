#pragma once

#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include "aruco.h"
#include "position.hpp"
#include "undistort_image.h"

using namespace std;
using namespace cv;
using namespace aruco;

class ArUcoProcessor {
 private:
  Ptr<Dictionary> dictionary;
  Ptr<MarkerDetector> detector;
  Ptr<MarkerPoseTracker> pose_tracker;

  CameraParameters cam_params_;
  float target_size_;
  Position calculate_pose(Marker& marker);

 public:
  ArUcoProcessor(CameraParameters cam_params, float targetSize);
  ArUcoProcessor(CameraParameters cam_params, float targetSize,
                 Ptr<MarkerDetector> detectorParameters);

  std::optional<Position> process_raw_frame(Mat image, int markerID = 19);
  Mat draw_markers_and_axis(Mat image, Marker marker);
};
