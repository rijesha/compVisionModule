#ifndef LOCATION_PROCESSOR_H
#define LOCATION_PROCESSOR_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <common/undistort_image.h>
#include "configuration.h"
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include <common/aruco_processor.h>
#include <unistd.h>

using namespace std;
using namespace cv;

class LocationProcessor
{
private:
    clock_t imageAcquisitionTime, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime;
    VideoCapture vCap;
    UndistortImage ui;
    Mat image;
    Mat original;
    ArUcoProcessor arProc;
    
public:
    LocationProcessor(string calib_file_path, int device_id);
    Position processImage(void);
};

#endif /* ARUCO_PROCESSOR_H */