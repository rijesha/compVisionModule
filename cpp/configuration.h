#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <opencv2/opencv.hpp>


Scalar MASK_1_LOWER_BOUND(1 , 255, 40 );
Scalar MASK_1_UPPER_BOUND(6 , 255, 255);
Scalar MASK_2_LOWER_BOUND(5 , 105, 130);
Scalar MASK_2_UPPER_BOUND(10, 255, 255);
Scalar MASK_3_LOWER_BOUND(10, 60 , 191);
Scalar MASK_3_UPPER_BOUND(20, 255, 255);

int CAMPARAMS_WIDTH = 640;
int CAMPARAMS_HEIGHT = 480;
Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 693.19269388, 0., 324.16405438, 0., 688.67070551, 246.27208573, 0., 0., 1.);
Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.45108306, 0.36336986, -0.00262423, -0.0026966, -0.24869988);

#endif /* CONFIGURATION_H */