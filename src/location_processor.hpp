#ifndef LOCATION_PROCESSOR_H
#define LOCATION_PROCESSOR_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "../aruco/undistort_image.h"
#include "configuration.h"
#include "../aruco/argparse/argparse.hpp"
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include "../aruco/aruco_processor.h"
#include <unistd.h>

using namespace std;


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