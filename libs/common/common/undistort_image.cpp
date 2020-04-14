#include "undistort_image.h"

UndistortImage::UndistortImage(){}

UndistortImage::UndistortImage(CameraParameters camparams){
    this->camparams = camparams;
    this->new_camera_matrix = getOptimalNewCameraMatrix(camparams.CameraMatrix, camparams.Distorsion, camparams.CamSize, 1, camparams.CamSize);
}

void UndistortImage::undistortAcquiredImage(Mat img, Mat *dstImg){
    undistort(img, *dstImg, camparams.CameraMatrix, camparams.Distorsion, this->new_camera_matrix);
}

Marker UndistortImage::undistortMarkerPoints(Marker src_pts){
    Marker dst_pts = Marker();
    undistortPoints(src_pts, dst_pts, camparams.CameraMatrix, camparams.Distorsion, Mat(), this->new_camera_matrix);
    return dst_pts;
}