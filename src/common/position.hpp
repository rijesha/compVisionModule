#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../libs/aruco/include/aruco/aruco.h"
//#include "configuration.h"
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
    Position(float x, float y, float depth, float yaw);

    Position(Mat RTmatrix);

    bool emptyPosition = false;
    bool isDesiredPosition = false;
    Mat rvecs, tvecs, RTMatrix;
    Mat eulersAngles;
    
    Mat rotMat;
    //Mat worldPos;
    string getInfoString();
    string getBasicString();
    clock_t creation_time;
    clock_t time_since_last_positon;
    
    float x,y,z;
    float ele, azi, tilt;
    float w_x, w_y, w_z;
    float angle_in_frame();

    bool A();
    bool B();
    bool C();
    bool D();
    bool E();
    bool F();
};

#endif 