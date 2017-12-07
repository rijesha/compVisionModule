#ifndef LOCATION_PROCESSOR_H
#define LOCATION_PROCESSOR_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "aruco/undistort_image.h"
#include "configuration.h"
#include "aruco/argparse/argparse.hpp"
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include "aruco/aruco_processor.h"
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
    bool processImage(void);

    
public:
    LocationProcessor();
    void LocationProcessingThread(void);
    bool switchToCloseInProcessing(bool);
    void Stop(void);
    bool shutdownFlag;
};

#endif /* ARUCO_PROCESSOR_H */