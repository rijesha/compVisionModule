
#include "aruco_processor.h"

ArUcoProcessor::ArUcoProcessor(CameraParameters cam_params, float target_size)
    : cam_params_(cam_params), target_size_(target_size) {
  detector = new MarkerDetector();
  pose_tracker = new MarkerPoseTracker();
  detector->setDictionary("ARUCO_MIP_16h3");
  MarkerDetector::Params& params = detector->getParameters();

  params.setDetectionMode(DM_FAST, 0.02);
  params.setCornerRefinementMethod(CORNER_LINES);

  // this->dictionary = getPredefinedDictionary(dictionaryName);

  // this->detectorParameters = new aruco::MarkerDetector;
  // detectorParameters->cornerRefinementMethod = CORNER_REFINE_SUBPIX;
  // detectorParameters->cornerRefinementWinSize = 5;
  // detectorParameters->cornerRefinementMinAccuracy = .001;
  // detectorParameters->cornerRefinementMaxIterations = 2000;
  // detectorParameters->adaptiveThreshWinSizeMin = 17;
  // detectorParameters->adaptiveThreshWinSizeMax = 17;
}

ArUcoProcessor::ArUcoProcessor(CameraParameters cam_params, float target_size,
                               Ptr<MarkerDetector> detectorParameters)
    : cam_params_(cam_params), target_size_(target_size) {
  // this->dictionary = getPredefinedDictionary(dictionaryName);
  // this->detectorParameters = detectorParameters;
  pose_tracker = new MarkerPoseTracker();
}

std::optional<Position> ArUcoProcessor::process_raw_frame(Mat image,
                                                          int markerID) {
  auto detectedMarkers = detector->detect(image);
  for (Marker m : detectedMarkers) {
    if (m.id == markerID) {
      return calculate_pose(m);
    }
  }
  return {};
}

Position ArUcoProcessor::calculate_pose(Marker& marker) {
  /*
  if (positiveYaw()) {
     // cout << "Positive YAW ";
  } else {
    //  cout << "Negative YAW ";
  }*/
  pose_tracker->estimatePose(marker, cam_params_, target_size_, 1.0);
  Mat RTmatrix = pose_tracker->getRTMatrix();
  // cout << "RVEC  " << endl;
  // cout << pose_tracker->getRvec() << endl;
  // cout << "done RVEC" << endl;
  if (!RTmatrix.empty()) {
    return Position(RTmatrix);
  }
  // cout << p.azi << endl;
  return {};
}

Mat ArUcoProcessor::draw_markers_and_axis(Mat image, Marker marker) {
  Mat out = image.clone();

  try {
    aruco::CvDrawingUtils::draw3dAxis(out, marker, cam_params_);
    aruco::CvDrawingUtils::draw3dCube(out, marker, cam_params_);
  } catch (...) {
  }

  return out;
}
