#ifndef LOCATION_PROCESSOR_H
#define LOCATION_PROCESSOR_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "../aruco/undistort_image.h"
//#include "configuration.h"
#include "../aruco/argparse/argparse.hpp"
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include "../aruco/aruco_processor.h"
#include <unistd.h>

using namespace std;
using namespace cv;

#define DEVICE_ID 0

#define TARGET_WIDTH 0.150
#define MARKER_ID 21


#define CAM_WIDTH 640
#define CAM_HEIGHT 480
//CameraParameters for width = 640, height = 480, with pixel rms = 0.29282934871924216 
Mat CAMPARAMS_CAMERA_MATRIX = (Mat_<double>(3,3) << 648.9598628012942, 0.0, 272.2112118006674, 0.0, 649.9002809370122, 222.7163833874802, 0.0, 0.0, 1.0);
Mat CAMPARAMS_DIST_COEFS = (Mat_<double>(1,5) << -0.44844610047416084,  0.42181868761795005, 0.0012724412556982193, 0.005997805659282252, -0.3984919931833452);


class LocationProcessor
{
private:
    clock_t imageAcquisitionTime, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime;
    VideoCapture vCap;
    UndistortImage ui;
    Mat image;
    Mat original;
    ArUcoProcessor arProc;
    bool processImage(void);
    bool shutdownFlag;
    void Stop(void);
public:
    LocationProcessor();
    bool LocationProcessingThread(void);
    bool switchToCloseInProcessing(bool);
};

#endif /* ARUCO_PROCESSOR_H */