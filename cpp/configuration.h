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

//Orig 4cm err
//Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 1368.90765, 0., 646.197774, 0., 1445.32276, 536.014605, 0., 0., 1.);
//Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.43027182,  0.26074604, -0.00464682, -0.00351217, -0.17564315);

//new 5cm err
//Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 1349.0609212188597, 0.0, 591.4195690135933, 0.0, 1436.1095238785215, 506.2012637025493, 0.0, 0.0, 1.0);
//Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.4216047189006637,  0.2670732929006295, 0.0009607337910424751, 0.0020335764906009917, -0.14399387537918165);

//new calib (USE THIS ONE!!!!) 1280 x 960
Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 1345.606454578346, 0.0, 615.7241339966613, 0.0, 1431.247134536588, 510.4200513208423, 0.0, 0.0, 1.0);
Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.4213890822236854,  0.24304334811195638, 0.0005357509298507465, -0.00029838491601875126, -0.0998885744205625);

//New new calib
//Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 1329.500200021851, 0.0, 629.1726337506328, 0.0, 1417.9656081327146, 499.42395669220366, 0.0, 0.0, 1.0);
//Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.4231804443205848,  0.2112018743143005, 0.002665606668676977, -0.0019431393915243344, 0.00776728912031021);

//5mpx width = 2592, height = 1944
//Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 2637.7516942470124, 0.0, 1236.0059152213103, 0.0, 2632.42111686543, 966.6824461681633, 0.0, 0.0, 1.0);
//Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.3997276419266211,  0.17709586322159893, 0.0009560836893463695, 0.0008219916461615755, -0.006913436016816682);

#endif /* CONFIGURATION_H */