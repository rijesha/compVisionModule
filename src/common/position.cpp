#include "position.hpp"

#define RESET_TIME 2

clock_t Position::last_creation_time = clock();

Position::Position(){
    emptyPosition = true;
}

Position::Position(double x, double y, double depth, double yaw){
    isDesiredPosition = true;
    this->x = x;
    this->y = y;
    this->depth = depth;
    this->azi = yaw;
}

Position::Position(Mat RTMatrix) {
    creation_time = clock();
    time_since_last_positon = creation_time - last_creation_time;
    last_creation_time = creation_time;
    emptyPosition = false;

    this->RTMatrix = RTMatrix(Range(0, 3), Range(0, 4));
    tvecs = RTMatrix(Range(0, 3), Range(3, 4));
    rotMat = RTMatrix(Range(0, 3), Range(0, 3));

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;

    decomposeProjectionMatrix( this->RTMatrix,
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulersAngles);

    worldPos = -rotMat.inv() * tvecs;

    float* _t = tvecs.ptr<float>();
    x = _t[0]; y = _t[1]; depth = _t[2];
    
    float* _eA = eulersAngles.ptr<float>();
    ele  = _eA[0]; azi  = _eA[1]; tilt = _eA[2];
}

string Position::getInfoString(){
    stringstream output;
    output << std::fixed;
    output << std::setprecision(5);

    output << x << ',' << y << ',' << depth << ',';
    output << ele << ',' << azi << ',' << tilt << ',';
    output << worldPos.at<double>(0) << ',' << worldPos.at<double>(1) << ',' << worldPos.at<double>(2) << endl;
    return output.str();
}

string Position::getBasicString(){
    stringstream output;
    output << std::fixed;
    output << std::setprecision(5);

    output << x << ',' << y << ',' << depth << ',' << azi << endl;
    return output.str();
}

double Position::angle_in_frame(){
    return atan(x/depth) * 180/3.14;
}

bool Position::A(){
    //Target Detection Failed for X seconds
    return time_since_last_positon > RESET_TIME*CLOCKS_PER_SEC;
}
bool Position::B(){
    //Target Within 6 m depth
    return depth < 6;
}
bool Position::C(){
    //Target Azimuthal Angle <60 deg
    return azi < 60;
}
bool Position::D(){
    //Within 2m depth and Target zimuthal angle < 10 deg
    return (azi < 10 && depth < 2);
}
bool Position::E(){
    //Sensor Contact
    return false;
}
bool Position::F(){
    //Data Acquisition Complete
    return false;
}