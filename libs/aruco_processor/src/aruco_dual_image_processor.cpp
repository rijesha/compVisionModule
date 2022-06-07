#include "aruco_processor/aruco_processor.h"

ArUcoDualImageProcessor::ArUcoDualImageProcessor(CameraParameters cam_params_1, CameraParameters cam_params_2, float target_size)
    : cam_params_1_(cam_params_1),cam_params_2_(cam_params_2), target_size_(target_size) {
  detector = new MarkerDetector();
  pose_tracker = new MarkerPoseTracker();
  detector->setDictionary("ARUCO_MIP_16h3");
  MarkerDetector::Params& params = detector->getParameters();

  params.setDetectionMode(DM_FAST, 0);
  params.setCornerRefinementMethod(CORNER_LINES);
}

std::optional<Marker> get_marker(Mat image, int marker_id){
  auto detectedMarkers = detector->detect(image);
  for (Marker m: detectedMarkers){
    if (m.id == markerID){
      return m;
    }
  }
  return {};
}

cv::Point3f calculate_marker_points_3d(cv::Point point_1, cv::Point point_2){
  int height = 800;
  int width = 848;
  int f = 295; // averaged from calib files
  float B = .064; //mm 
  int disparity = point2.x - point_1.x;

  float Z = f*B/disparity;
  float X = (col-width/2.0)*Z/f;
  float Y = (height/2.0-row)*Z/f;
  return {X,Y,Z};
}

void process_points(vector<cv::Point> points1, vector<cv::Point> points2){
  vector<cv::Point3f> points_real_space;
  for (int i  = 0 ; i <4; i++){
    points_real_space.push_back(calculate_marker_points_3d(points1[i],points2[i]));
  }
  
  auto a = points_real_space[0]- points_real_space[1];
  auto b = points_real_space[2]- points_real_space[1];

  auto i = a.y*b.z - a.z*b.y;
  auto j = -( a.x*b.z - a.z*b.x);
  auto k = a.x*b.y - a.y*b.x;


  auto d = i*points_real_space[1].x +j*points_real_space[1].y+k*points_real_space[1].z;

  cout << "plane calc" << i << ", "<< j << ", "<< k << endl;
}

std::optional<std::pair< Marker, ArucoPosition>>
ArUcoDualImageProcessor::process_raw_frame(Mat image1, Mat image2, int markerID) {
  auto res1 = get_marker(image1, markerID);
  auto res2 = get_marker(image2, markerID);

  if (res1.has_value() && res2.has_value()){
    //calculate pose
    process_points(res1.value().contourPoints, res2.value().contourPoints);

    auto res =
      pose_tracker->estimatePose(res1.value(), cam_params_, target_size_, 10.0);
    
    cout << "matrix calc" << m.rvec[0] << ", "<< m.rvec[1] << ", "<< m.rvec[2] << endl;
    Mat RTmatrix = pose_tracker->getRTMatrix();
    if (!RTmatrix.empty()) {
      return {res1.value(), ArucoPosition(RTmatrix)};
    }
  
    //cv::Mat _rvec;
    //__aruco_solve_pnp(Marker::get3DPoints(target_size_), res1.value(), _cam_params.CameraMatrix, _cam_params.Distorsion, _rvec,  _tvec);
    //_rvec.convertTo(m.Rvec,CV_32F);
    //_tvec.convertTo(m.Tvec,CV_32F);
  }
  return {};
}

std::optional<ArucoPosition> ArUcoDualImageProcessor::calculate_pose(Marker& marker) {
  /*
  if (positiveYaw()) {
     // cout << "Positive YAW ";
  } else {
    //  cout << "Negative YAW ";
  }*/
  auto res =
      pose_tracker->estimatePose(marker, cam_params_, target_size_, 10.0);
  Mat RTmatrix = pose_tracker->getRTMatrix();
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

Mat ArUcoDualImageProcessor::draw_markers_and_axis(Mat image, Marker marker) {
  Mat out = image.clone();

  try {
    aruco::CvDrawingUtils::draw3dAxis(out, marker, cam_params_);
    aruco::CvDrawingUtils::draw3dCube(out, marker, cam_params_);
  } catch (...) {
  }

  return out;
}
