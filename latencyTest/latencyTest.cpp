#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "configuration.h"
#include <ctime>
#include <chrono>
#include <time.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <unistd.h>

using namespace std;
using namespace cv;

VideoCapture vCap;
Mat image;
auto startTime = std::chrono::high_resolution_clock::now();
bool shutdown = false;

double getTimeSinceStart(){
    return ((std::chrono::duration<double>) (std::chrono::high_resolution_clock::now() - startTime)).count();
}

void displayTimeToConsole()
{
    uint sleeptime = 1000000 * CONSOLESLEEP_SECONDS;
    while (!shutdown){
        usleep(sleeptime);
        cout << getTimeSinceStart() << endl;
    }
}

int main(int argc, const char** argv )
{   
    thread t1(displayTimeToConsole);
    
    vCap = VideoCapture(DEVICE_ID);
    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);

    vCap.read(image);

    while(1) {
        vCap.read(image);
        float currenttime = getTimeSinceStart();
        imwrite( "testImages/" + to_string(currenttime) + ".jpg", image );
        usleep(250000);
    }
    shutdown = false;
    t1.join();

    return 0;
}