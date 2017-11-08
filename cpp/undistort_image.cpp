#include "undistort_image.h"

UndistortImage::UndistortImage(){}

UndistortImage::UndistortImage(CameraParameters camparams){
    this->camparams = camparams;  
    this->camparams.new_camera_matrix = getOptimalNewCameraMatrix(camparams.camera_matrix, camparams.dist_coefs, cvSize(camparams.width,camparams.height), 1, cvSize(camparams.width,camparams.height));
}

void UndistortImage::undistortAcquiredImage(Mat img, Mat dstImg){
    undistort(img, dstImg, camparams.camera_matrix, camparams.dist_coefs, camparams.new_camera_matrix );
}
    