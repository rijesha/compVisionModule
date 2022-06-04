#include "aruco_processor/aruco_dual_image_processor.h"

ArUcoDualImageProcessor::ArUcoDualImageProcessor(CameraParameters cam_params_1,
                                                 CameraParameters cam_params_2,
                                                 float target_size)
    : cam_params_1_(cam_params_1),
      cam_params_2_(cam_params_2),
      target_size_(target_size) {
  detector1 = new MarkerDetector();
  detector2 = new MarkerDetector();
  pose_tracker1 = new MarkerPoseTracker();
  pose_tracker2 = new MarkerPoseTracker();
  detector1->setDictionary("ARUCO_MIP_16h3");
  detector2->setDictionary("ARUCO_MIP_16h3");

  MarkerDetector::Params& params1 = detector1->getParameters();

  params1.setDetectionMode(DM_FAST, 0);
  params1.setCornerRefinementMethod(CORNER_SUBPIX);

  MarkerDetector::Params& params2 = detector2->getParameters();

  params2.setDetectionMode(DM_FAST, 0);
  params2.setCornerRefinementMethod(CORNER_SUBPIX);
}

std::optional<Marker> ArUcoDualImageProcessor::get_marker(
    Mat image, Ptr<MarkerDetector> detector, int marker_id) {
  auto detectedMarkers = detector->detect(image);
  for (Marker m : detectedMarkers) {
    if (m.id == marker_id) {
      return m;
    }
  }
  return {};
}

cv::Point3f ArUcoDualImageProcessor::calculate_marker_points_3d(
    cv::Point2f point_1, cv::Point2f point_2) {
  int height = 800;
  int width = 848;
  int f = 286;     // averaged from calib files
  float B = .064;  // m
  float disparity = -point_2.x + point_1.x;
  float Z = f * B / disparity;
  // cout << "raw x1,y1, x2,y2: " << point_1.x << " " << point_1.y << " "
  //     << point_2.x << " " << point_2.y << " disparity: " << disparity
  //     << " z: " << Z << endl;

  float X = (point_1.x - width / 2.0) * Z / f;
  float Y = (height / 2.0 - point_1.y) * Z / f;
  return {X, Y, Z};
}

float ArUcoDualImageProcessor::calculate_yaw_via_depth(Marker& marker1,
                                                       Marker& marker2) {
  vector<cv::Point3f> points_real_space;
  for (int i = 0; i < 4; i++) {
    points_real_space.push_back(
        calculate_marker_points_3d(marker1[i], marker2[i]));
  }

  current_run_time = std::chrono::high_resolution_clock::now();
  timestamp = (current_run_time - start_time).count();

  DataPoint sensor_data;
  VectorXd lidar_vec(NZ_LIDAR);
  lidar_vec << points_real_space[1].z - points_real_space[0].z,
      points_real_space[1].x - points_real_space[0].x;
  sensor_data.set(timestamp / 1000, DataPointType::LIDAR, lidar_vec);
  VectorXd prediction;
  fusionUKF.process(sensor_data);
  prediction = fusionUKF.get();

  float yaw = -atan2(prediction[0], prediction[1]);

  cout << "x,z: " << points_real_space[1].x - points_real_space[0].x << " "
       << points_real_space[1].z - points_real_space[0].z
       << "yaw: " << yaw * 180 / 3.14 << endl;
  return yaw * 180 / 3.14;
}

std::optional<std::pair<std::pair<Marker, Marker>, ArucoPosition>>
ArUcoDualImageProcessor::process_raw_frame(Mat image1, Mat image2,
                                           int markerID) {
  auto res1 = get_marker(image1, detector1, markerID);
  auto res2 = get_marker(image2, detector2, markerID);
  if (res1.has_value() && res2.has_value()) {
    // calculate pose

    auto yaw = calculate_yaw_via_depth(res1.value(), res2.value());
    auto solutions =
        aruco::solvePnP_(Marker::get3DPoints(target_size_), res1.value(),
                         cam_params_1_.CameraMatrix, cam_params_1_.Distorsion);

    auto calculated_position_1 = ArucoPosition(solutions[0].first);
    auto calculated_position_2 = ArucoPosition(solutions[1].first);

    cout << "yaw_check: " << yaw << " "
         << calculated_position_1.target_ned_vector.yaw << " "
         << calculated_position_1.target_ned_vector.y << " "
         << calculated_position_2.target_ned_vector.yaw << " "
         << calculated_position_2.target_ned_vector.y << " " << endl;

    //    auto res = pose_tracker1->estimatePose(res1.value(),
    //    cam_params_1_,
    //                                         target_size_, 10.0);

    auto res3 = pose_tracker2->estimatePose(res2.value(), cam_params_2_,
                                            target_size_, 10.0);
    // process_points(res1.value(), res2.value());

    // auto solutions =
    //   solvePnP(Marker::get3DPoints(target_size_), res1.has_value(),
    //           cam_params_1_.CameraMatrix, cam_params_1_.Distorsion);

    Mat RTmatrix = solutions[0].first;
    if (!RTmatrix.empty()) {
      return std::make_pair(std::make_pair(res1.value(), res2.value()),
                            ArucoPosition(RTmatrix));
    }

    // cv::Mat _rvec;
    //__aruco_solve_pnp(Marker::get3DPoints(target_size_), res1.value(),
    //_cam_params.CameraMatrix, _cam_params.Distorsion, _rvec,  _tvec);
    //_rvec.convertTo(m.Rvec,CV_32F);
    //_tvec.convertTo(m.Tvec,CV_32F);
  }
  return {};
}

std::optional<ArucoPosition> ArUcoDualImageProcessor::calculate_pose(
    Marker& marker) {
  /*
  if (positiveYaw()) {
     // cout << "Positive YAW ";
  } else {
    //  cout << "Negative YAW ";
  }*/
  auto res =
      pose_tracker1->estimatePose(marker, cam_params_1_, target_size_, 10.0);
  Mat RTmatrix = pose_tracker1->getRTMatrix();
  /*
  if (res) {
    cout << "RT MAT " << pose_tracker->getRTMatrix() << endl;
  }

  cout << "RVEC  " << endl;
  cout << res << endl;
  cout << marker.Rvec << endl;
  cout << marker.Tvec << endl;
  // cout << marker.getTransformMatrix() << endl;
  cout << "done RVEC" << endl;
  */

  if (!RTmatrix.empty()) {
    return ArucoPosition(RTmatrix);
  }
  // cout << p.azi << endl;
  return {};
}

Mat ArUcoDualImageProcessor::draw_markers_and_axis(Mat image, Marker marker,
                                                   int cam_num) {
  Mat out = image.clone();

  try {
    if (cam_num == 1) {
      aruco::CvDrawingUtils::draw3dAxis(out, marker, cam_params_1_);
      aruco::CvDrawingUtils::draw3dCube(out, marker, cam_params_1_);
    } else {
      aruco::CvDrawingUtils::draw3dAxis(out, marker, cam_params_2_);
      aruco::CvDrawingUtils::draw3dCube(out, marker, cam_params_2_);
    }

  } catch (...) {
  }

  return out;
}
