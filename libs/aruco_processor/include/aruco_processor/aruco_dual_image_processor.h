#pragma once

#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include "aruco.h"
#include "aruco_processor/aruco_position.h"
#include "common/load_fisheye_calibration.h"
#include "ippe.h"
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

  FusionUKF ukf_pt1;
  FusionUKF ukf_pt2;
  FusionUKF ukf_pt3;
  FusionUKF ukf_pt4;

  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      start_time = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      current_run_time = std::chrono::high_resolution_clock::now();

  long timestamp = 0;

  FisheyeParams params_;
  CameraParameters cam_params_1_;
  CameraParameters cam_params_2_;
  float target_size_;
  bool enable_tracking_{true};

  std::optional<Marker> get_marker(Mat image, Ptr<MarkerDetector> detector,
                                   int marker_id);
  cv::Point3f calculate_marker_points_3d(cv::Point2f point_1,
                                         cv::Point2f point_2);
  std::pair<float, float> calculate_yaw_via_depth(Marker& marker1,
                                                  Marker& marker2);

 public:
  ArUcoDualImageProcessor(FisheyeParams params, CameraParameters cam_params_1,
                          CameraParameters cam_params_2, float targetSize, bool enable_tracking = true);

  std::optional<std::pair<std::pair<Marker, Marker>, std::pair<ArucoPosition,ArucoPosition>>>
  process_raw_frame(Mat image1, Mat image2, int markerID);
  Mat undistort_left_image(Mat& img);
  Mat undistort_right_image(Mat& img);
  Mat draw_markers_and_axis(Mat image, Marker marker, int cam_number);
};
