#ifndef LOCATION_PROCESSOR_H
#define LOCATION_PROCESSOR_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "common/undistort_image.h"
#include "common/configuration.h"
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include <common/aruco_processor.h>
#include <unistd.h>
#include <camera-v4l2/camera.h>


using namespace std;
using namespace cv;

class LocationProcessor
{
private:
    clock_t imageAcquisitionTime, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime;
    Camera * camera;
    UndistortImage ui;
    Mat image;
    Image image1;
    int width, height;
    
public:
    LocationProcessor(string calib_file_path, string device_id, int gain, int exposure);
    Position processImage(void);
    Mat original;
    ArUcoProcessor arProc;
};

#endif /* ARUCO_PROCESSOR_H */