#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "configuration.h"
#include <ctime>
#include <chrono>

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
    Position(vector< Vec3d > rvecs, vector< Vec3d > tvecs);
    bool emptyPosition = false;
    vector< Vec3d > rvecs, tvecs;
    Vec3d eulersAngles;
    Mat rotMat;
    Mat worldPos;
    string getInfoString();
    clock_t creation_time;
    clock_t time_since_last_positon;
    
    double x,y,depth;
    double ele, azi, tilt;

    bool A();
    bool B();
    bool C();
    bool D();
    bool E();
    bool F();
};

#endif 