#include "aruco_processor/aruco_dual_image_processor.h"

ArUcoDualImageProcessor::ArUcoDualImageProcessor(FisheyeParams params,
                                                 CameraParameters cam_params_1,
                                                 CameraParameters cam_params_2,
                                                 float target_size,
                                                 bool enable_tracking)
    : params_(params),
      cam_params_1_(cam_params_1),
      cam_params_2_(cam_params_2),
      target_size_(target_size),
      enable_tracking_(enable_tracking) {
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
  float disparity = point_1.x - point_2.x;
  float x = (point_1.x + point_2.x) / 2.0;
  float y = (point_1.y + point_2.y) / 2.0;
  // cout << "disparity: " << disparity
  //     << " Z: " << 38.790829404744855 * .064 / disparity << endl;
  return {x, y, disparity};
}

cv::Point3f normalize(cv::Point3f pt) {
  float mag = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
  return pt / mag;
}

void calculate_yaw_via_coplanar(vector<cv::Point3f> points_real_space) {
  cv::Point3f v10 = points_real_space[1] - points_real_space[0];
  cv::Point3f v21 = points_real_space[2] - points_real_space[1];
  cv::Point3f v32 = points_real_space[3] - points_real_space[2];
  cv::Point3f v03 = points_real_space[0] - points_real_space[3];

  cv::Point3f r0 = normalize(v10.cross(v21));
  cv::Point3f r1 = normalize(v21.cross(v32));
  cv::Point3f r2 = normalize(v32.cross(v03));
  cv::Point3f r3 = normalize(v03.cross(v10));

  // cout << "r0: " << r0 << endl;
  // cout << "r1: " << r1 << endl;
  // cout << "r2: " << r2 << endl;
  // cout << "r3: " << r3 << endl;

  cout << "r: " << (r0 + r1 + r2 + r3) / 4 << endl;
}

std::pair<float, float> ArUcoDualImageProcessor::calculate_yaw_via_depth(
    Marker& marker1, Marker& marker2) {
  vector<cv::Point2f> undistorted_marker_1;
  vector<cv::Point2f> undistorted_marker_2;
  cv::fisheye::undistortPoints(marker1, undistorted_marker_1, params_.K1,
                               params_.D1, params_.R1, params_.P1);

  cv::fisheye::undistortPoints(marker2, undistorted_marker_2, params_.K2,
                               params_.D2, params_.R2, params_.P2);

  vector<cv::Point3f> disparity_points;
  for (int i = 0; i < 4; i++) {
    disparity_points.push_back(calculate_marker_points_3d(
        undistorted_marker_1[i], undistorted_marker_2[i]));
  }
  vector<cv::Point3f> points_real_space;
  cv::perspectiveTransform(disparity_points, points_real_space, params_.Q);
  // cout << "3d points" << points_real_space << endl;

  // calculate_yaw_via_coplanar(points_real_space);
  current_run_time = std::chrono::high_resolution_clock::now();
  timestamp = (current_run_time - start_time).count();

  auto z_delta = ((points_real_space[1].z - points_real_space[0].z) +
                  (points_real_space[2].z - points_real_space[3].z)) /
                 2;

  cout << " " << z_delta << " "
       << (points_real_space[1].z - points_real_space[0].z) << " "
       << (points_real_space[2].z - points_real_space[3].z) << endl;

  auto x_delta = ((points_real_space[1].x - points_real_space[0].x) +
                  (points_real_space[2].x - points_real_space[3].x)) /
                 2;

  float yaw1 = -atan2(points_real_space[1].z - points_real_space[0].z,
                      points_real_space[1].x - points_real_space[0].x);

  float yaw2 = -atan2(points_real_space[2].z - points_real_space[3].z,
                      points_real_space[2].x - points_real_space[3].x);

  float pitch1 = -atan2(points_real_space[3].z - points_real_space[0].z,
                        -points_real_space[3].y + points_real_space[0].y);

  float pitch2 = -atan2(points_real_space[2].z - points_real_space[1].z,
                        -points_real_space[2].y + points_real_space[1].y);

  DataPoint sensor_data;
  VectorXd lidar_vec(NZ_LIDAR);
  // lidar_vec << x_delta, z_delta;
  lidar_vec << (pitch1 + pitch2) * .180 / 6.28, (yaw1 + yaw2) * .180 / 6.28;
  sensor_data.set(timestamp / 1000, DataPointType::LIDAR, lidar_vec);
  VectorXd prediction;
  ukf_pt1.process(sensor_data);
  prediction = ukf_pt1.get();

  /*
    cout << "yaw: " << yaw1 * 180 / 3.14 << " " << yaw2 * 180 / 3.14 << " "
         << prediction[1] * 1000 << endl;
    cout << "pitch: " << pitch1 * 180 / 3.14 << " " << pitch2 * 180 / 3.14 << "
    "
         << prediction[0] * 1000 << endl;
  */

  // cout << "x,z: " << points_real_space[1].x - points_real_space[0].x << " "
  //     << points_real_space[1].z - points_real_space[0].z
  //     << "yaw: " << yaw * 180 / 3.14 << endl;
  return std::make_pair(prediction[1] * 1000, prediction[0] * 1000);
}

void impl__aruco_getRTfromMatrix44(const cv::Mat& M, cv::Mat& R, cv::Mat& T) {
  assert(M.cols == M.rows && M.cols == 4);
  assert(M.type() == CV_32F || M.type() == CV_64F);
  // extract the rotation part
  cv::Mat r33 = cv::Mat(M, cv::Rect(0, 0, 3, 3));
  cv::SVD svd(r33);
  cv::Mat Rpure = svd.u * svd.vt;
  cv::Rodrigues(Rpure, R);
  T.create(1, 3, M.type());
  if (M.type() == CV_32F)
    for (int i = 0; i < 3; i++) T.ptr<float>(0)[i] = M.at<float>(i, 3);
  else
    for (int i = 0; i < 3; i++) T.ptr<double>(0)[i] = M.at<double>(i, 3);
}

 std::optional<std::pair<std::pair<Marker, Marker>, std::pair<ArucoPosition,ArucoPosition>>>
ArUcoDualImageProcessor::process_raw_frame(Mat image1, Mat image2,
                                           int markerID) {
  auto res1 = get_marker(image1, detector1, markerID);
  auto res2 = get_marker(image2, detector2, markerID);
  if (res1.has_value() && res2.has_value()) {
    // calculate pose

    //    vector<cv::Point2f> undistorted_marker_1;
    //  cv::fisheye::undistortPoints(res1.value(), undistorted_marker_1,
    //  params_.K1,
    //                             params_.D1, params_.R1, params_.P1);

    pose_tracker2->estimatePose(res2.value(), cam_params_2_, target_size_,
                                10.0);

    if (enable_tracking_) {
      pose_tracker1->estimatePose(res1.value(), cam_params_1_, target_size_,
                                  10.0);
      return std::make_pair(
          std::make_pair(res1.value(), res2.value()),
          std::make_pair(ArucoPosition(pose_tracker1->getRTMatrix()),
                         ArucoPosition(pose_tracker2->getRTMatrix())));

    } else {
      auto solutions = aruco::solvePnP_(
          Marker::get3DPoints(target_size_), res1.value(),
          cam_params_1_.CameraMatrix, cam_params_1_.Distorsion);
      // auto solutions = aruco::solvePnP_(
      //    Marker::get3DPoints(target_size_), undistorted_marker_1,
      //    Mat::eye(3, 3, CV_32F), Mat::zeros(1, 5, CV_32F));

      vector<ArucoPosition> possible_solutions;
      possible_solutions.push_back(ArucoPosition(solutions[0].first));
      possible_solutions.push_back(ArucoPosition(solutions[1].first));

      int solution_index = 0;
      int incorrect_solution_index = 1;
      auto yaw_and_pitch = calculate_yaw_via_depth(res1.value(), res2.value());

      if (abs(possible_solutions[0].target_ned_vector.yaw -
              yaw_and_pitch.first) <
          abs(possible_solutions[1].target_ned_vector.yaw -
              yaw_and_pitch.first)) {
        solution_index = 0;
        incorrect_solution_index = 1;

      } else {
        solution_index = 1;
        incorrect_solution_index = 0;
      }

      solution_index = 0;
      /*
      cout << "yaw_check: " << yaw_and_pitch.first << " "
           << possible_solutions[solution_index].target_ned_vector.yaw << " "
           << possible_solutions[incorrect_solution_index].target_ned_vector.yaw
           << " " << endl;
      cout << "pitch_check: " << yaw_and_pitch.second << " "
           << possible_solutions[solution_index].ele << " "
           << possible_solutions[incorrect_solution_index].ele << " " << endl;
  */

      Mat rvec, tvec;
      impl__aruco_getRTfromMatrix44(solutions[solution_index].first, rvec,
                                    tvec);
      rvec.convertTo(res1.value().Rvec, CV_32F);
      tvec.convertTo(res1.value().Tvec, CV_32F);
      res1.value().ssize = target_size_;
      cout << "rvec: " << rvec << endl;

      return std::make_pair(
          std::make_pair(res1.value(), res2.value()),
          std::make_pair(possible_solutions[solution_index],
                         ArucoPosition(pose_tracker2->getRTMatrix())));
    }
  }
  return {};
}

Mat ArUcoDualImageProcessor::undistort_left_image(Mat& img) {
  Mat ret_img;
  cv::Matx33d newK = params_.K1;
  cv::fisheye::undistortImage(img, ret_img, params_.K1, params_.D1, newK);
  return ret_img;
}
Mat ArUcoDualImageProcessor::undistort_right_image(Mat& img) {
  Mat ret_img;
  cv::Matx33d newK = params_.K2;
  cv::fisheye::undistortImage(img, ret_img, params_.K2, params_.D2, newK);
  return ret_img;
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
