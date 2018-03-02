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

Position::Position(vector< Vec3d > rvecs, vector< Vec3d > tvecs) : rvecs(rvecs), tvecs(tvecs){
    creation_time = clock();
    time_since_last_positon = creation_time - last_creation_time;
    last_creation_time = creation_time;
    emptyPosition = false;
    Rodrigues(rvecs[0],rotMat);
    calcEulerAngles();
    Mat invRotMat;
    transpose(rotMat, invRotMat);
    worldPos = -invRotMat * Mat(tvecs[0]);
    x = tvecs[0][0];
    y = tvecs[0][1];
    depth = tvecs[0][2];
    
    ele  = eulersAngles[0];
    azi  = eulersAngles[1];
    tilt = eulersAngles[2];  
}

void Position::calcEulerAngles(){
    
    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotMat.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulersAngles);
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