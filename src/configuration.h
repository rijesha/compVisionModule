#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <opencv2/opencv.hpp>
using namespace cv;

//#define DISPLAY_IMAGE

#define DEVICE_ID 0

#define TARGET_WIDTH 0.150
#define MARKER_ID 21

//CameraParameters for width = 640, height = 480, with pixel rms = 0.29282934871924216
#define CAM_WIDTH 640
#define CAM_HEIGHT 480

#define CM_0 648.9598628012942
#define CM_1 0.0
#define CM_2 272.2112118006674
#define CM_3 0.0
#define CM_4 649.9002809370122
#define CM_5 222.7163833874802
#define CM_6 0.0
#define CM_7 0.0
#define CM_8 1.0

#define DC_0 -0.44844610047416084
#define DC_1 0.42181868761795005
#define DC_2 0.0012724412556982193
#define DC_3 0.005997805659282252
#define DC_4 -0.3984919931833452

#endif /* CONFIGURATION_H */
