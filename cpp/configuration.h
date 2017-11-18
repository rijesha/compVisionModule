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
Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 1368.90765, 0., 646.197774, 0., 1445.32276, 536.014605, 0., 0., 1.);
Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.43027182,  0.26074604, -0.00464682, -0.00351217, -0.17564315);



#endif /* CONFIGURATION_H */