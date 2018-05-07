#include "undistort_image.h"

UndistortImage::UndistortImage(){}

UndistortImage::UndistortImage(CameraParameters camparams){
    this->camparams = camparams;
    }

void UndistortImage::undistortAcquiredImage(Mat img, Mat *dstImg){
    undistort(img, *dstImg, camparams.CameraMatrix, camparams.Distorsion);
}
    