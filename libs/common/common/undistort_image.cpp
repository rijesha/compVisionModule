#include "undistort_image.h"

UndistortImage::UndistortImage(CameraParameters& camparams)
    : cam_params_(camparams) {
  new_camera_matrix_ = getOptimalNewCameraMatrix(
      cam_params_.CameraMatrix, cam_params_.Distorsion, cam_params_.CamSize, 1,
      cam_params_.CamSize);
}

void UndistortImage::undistort_acquired_image(Mat img, Mat* dstImg) {
  undistort(img, *dstImg, cam_params_.CameraMatrix, cam_params_.Distorsion,
            new_camera_matrix_);
}

Marker UndistortImage::undistort_marker_points(Marker src_pts) {
  Marker dst_pts = Marker();
  undistortPoints(src_pts, dst_pts, cam_params_.CameraMatrix,
                  cam_params_.Distorsion, Mat(), new_camera_matrix_);
  return dst_pts;
}
