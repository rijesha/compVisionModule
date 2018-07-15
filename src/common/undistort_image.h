#ifndef UNDISTORTIMAGE_H
#define UNDISTORTIMAGE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../libs/aruco/include/aruco/aruco.h"

using namespace std;
using namespace cv;
using namespace aruco;

class UndistortImage
{
private:
    CameraParameters camparams;
    Mat new_camera_matrix;
public:
    UndistortImage();
    UndistortImage(CameraParameters camparams);
    void undistortAcquiredImage(Mat img, Mat *dstImg);
};

#endif /* UNDISTORTIMAGE_H */
