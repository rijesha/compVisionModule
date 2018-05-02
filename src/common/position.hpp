#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../libs/aruco/include/aruco/aruco.h"
//#include "configuration.h"
#include <ctime>
#include <chrono>
#include <math.h> 

using namespace std;
using namespace cv;
using namespace aruco;

class Position
{
private:
    void calcEulerAngles();
    static clock_t last_creation_time;
public:
    Position();
    Position(double x, double y, double depth, double yaw);
    Position(vector< Vec3d > rvecs, vector< Vec3d > tvecs);
    bool emptyPosition = false;
    bool isDesiredPosition = false;
    vector< Vec3d > rvecs, tvecs;
    Vec3d eulersAngles;
    Mat rotMat;
    Mat worldPos;
    string getInfoString();
    string getBasicString();
    clock_t creation_time;
    clock_t time_since_last_positon;
    
    double x,y,depth;
    double ele, azi, tilt;
    double angle_in_frame();

    bool A();
    bool B();
    bool C();
    bool D();
    bool E();
    bool F();
};

#endif 