#ifndef UNDISTORTIMAGE_H
#define UNDISTORTIMAGE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../libs/aruco/include/aruco/aruco.h"

using namespace std;
using namespace cv;
using namespace aruco;

struct CameraParametersLocal {
    int width;
    int height;
    Mat camera_matrix;
    Mat dist_coefs;
    Mat new_camera_matrix;
};

class UndistortImage
{
private:
    CameraParameters camparams;
public:
    UndistortImage();
    UndistortImage(CameraParameters camparams);
    Mat undistortAcquiredImage(Mat img, Mat dstImg);
};

#endif /* UNDISTORTIMAGE_H */
