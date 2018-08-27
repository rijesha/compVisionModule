#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <opencv2/opencv.hpp>
#include "../libs/aruco/include/aruco/aruco.h"
using namespace cv;
using namespace aruco;

#define PRINT_INFO_STRING
#define DISPLAY_IMAGE 1

#define TARGET_WIDTH 0.158

#define TARGET_ANGLE -90
#define TARGET_ANGLE_RAD TARGET_ANGLE*3.14/180


#define FINAL_Z_ARUCO 1.08
#define FINAL_Y_ARUCO .05

#endif /* CONFIGURATION_H */
