#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <opencv2/opencv.hpp>

#define DISPLAY_IMAGE

#define CAM_WIDTH 1280
#define CAM_HEIGHT 960

#define TARGET_WIDTH 0.150

Scalar MASK_1_LOWER_BOUND(1 , 255, 40 );
Scalar MASK_1_UPPER_BOUND(6 , 255, 255);
Scalar MASK_2_LOWER_BOUND(5 , 105, 130);
Scalar MASK_2_UPPER_BOUND(10, 255, 255);
Scalar MASK_3_LOWER_BOUND(10, 60 , 191);
Scalar MASK_3_UPPER_BOUND(20, 255, 255);

int CAMPARAMS_WIDTH = 1280;
int CAMPARAMS_HEIGHT = 960;

Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 1349.0609212188597, 0.0, 591.4195690135933, 0.0, 1436.1095238785215, 506.2012637025493, 0.0, 0.0, 1.0);
Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.4216047189006637,  0.2670732929006295, 0.0009607337910424751, 0.0020335764906009917, -0.14399387537918165);


#endif /* CONFIGURATION_H */