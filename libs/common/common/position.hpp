#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "aruco.h"
#include <ctime>
#include <chrono>
#include <math.h>
#include <iomanip> 

using namespace std;
using namespace cv;
using namespace aruco;

class Position
{
private:
    static clock_t last_creation_time;
public:
    Position();
    Position(float x, float y, float z, float yaw);

    Position(Mat RTmatrix);

    bool emptyPosition = false;
    bool isDesiredPosition = false;
    Mat rvecs, tvecs, RTMatrix;
    Mat eulersAngles;
    
    Mat rotMat;
    string getInfoString();
    string getBasicString();
    clock_t creation_time;
    clock_t time_since_last_positon;
    
    float x = 0, y = 0, z = 0;
    float ele = 0, azi = 0, tilt = 0;
    float w_x = 0, w_y = 0, w_z = 0;
    float angle_in_frame();
    
};

#endif 