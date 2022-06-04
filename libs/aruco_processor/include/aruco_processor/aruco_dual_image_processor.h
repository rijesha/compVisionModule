#pragma once

#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include "aruco.h"
#include "ippe.h"
#include "aruco_processor/aruco_position.h"
#include "undistort_image.h"

#include "datapoint.h"
#include "fusionukf.h"

using namespace std;
using namespace cv;
using namespace aruco;

class ArUcoDualImageProcessor {
 private:
  Ptr<Dictionary> dictionary;
  Ptr<MarkerDetector> detector1;
  Ptr<MarkerDetector> detector2;
  Ptr<MarkerPoseTracker> pose_tracker1;
  Ptr<MarkerPoseTracker> pose_tracker2;

  FusionUKF fusionUKF;
  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      start_time = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      current_run_time = std::chrono::high_resolution_clock::now();

  long timestamp = 0;

  CameraParameters cam_params_1_;
  CameraParameters cam_params_2_;
  float target_size_;
  std::optional<ArucoPosition> calculate_pose(Marker& marker);

  std::optional<Marker> get_marker(Mat image, Ptr<MarkerDetector> detector,
                                   int marker_id);
  cv::Point3f calculate_marker_points_3d(cv::Point2f point_1,
                                         cv::Point2f point_2);
  float calculate_yaw_via_depth(Marker& marker1,
                                                         Marker& marker2);

 public:
  ArUcoDualImageProcessor(CameraParameters cam_params_1,
                          CameraParameters cam_params_2, float targetSize);

  std::optional<std::pair<std::pair<Marker, Marker>, ArucoPosition>>
  process_raw_frame(Mat image1, Mat image2, int markerID);
  Mat draw_markers_and_axis(Mat image, Marker marker, int cam_number);
};
