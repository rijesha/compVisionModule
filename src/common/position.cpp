#include "position.hpp"

#define RESET_TIME 2

clock_t Position::last_creation_time = clock();

Position::Position(){
    emptyPosition = true;
}

Position::Position(float x, float y, float z, float yaw){
    isDesiredPosition = true;
    this->x = x;
    this->y = y;
    this->z = z;
    this->azi = yaw;
}

Position::Position(Mat RTMatrixs) {
    creation_time = clock();
    time_since_last_positon = creation_time - last_creation_time;
    last_creation_time = creation_time;
    emptyPosition = false;

    RTMatrix = RTMatrixs(Range(0, 3), Range(0, 4));
    tvecs = RTMatrixs(Range(0, 3), Range(3, 4));
    rotMat = RTMatrixs(Range(0, 3), Range(0, 3));

    x = tvecs.at<float>(0); y = tvecs.at<float>(1); z = tvecs.at<float>(2);

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;

    decomposeProjectionMatrix( RTMatrix,
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulersAngles);


    Mat worldPos(-rotMat.inv() * tvecs);
    float* _wp = worldPos.ptr<float>();
    w_x  = _wp[0]; w_y = _wp[1]; w_z  = _wp[2];

    double* _eA = eulersAngles.ptr<double>();
    ele  = _eA[0]; azi  = _eA[1]; tilt = _eA[2];
}

string Position::getInfoString(){
    stringstream output;
    output << std::fixed;
    output << std::setprecision(5);

    output << x << ',' << y << ',' << z << ',';
    output << ele << ',' << azi << ',' << tilt << ',';
    output << w_x << ',' << w_y << ',' << w_z << endl;
    return output.str();
}

string Position::getBasicString(){
    stringstream output;
    output << std::fixed;
    output << std::setprecision(5);

    output << x << ',' << y << ',' << z << ',' << azi << endl;
    return output.str();
}

float Position::angle_in_frame(){
    return atan(x/z) * 180/3.14;
}

bool Position::A(){
    //Target Detection Failed for X seconds
    return time_since_last_positon > RESET_TIME*CLOCKS_PER_SEC;
}
bool Position::B(){
    //Target Within 6 m depth
    return y < 6;
}
bool Position::C(){
    //Target Azimuthal Angle <60 deg
    return azi < 60;
}
bool Position::D(){
    //Within 2m depth and Target zimuthal angle < 10 deg
    return (azi < 10 && y < 2);
}
bool Position::E(){
    //Sensor Contact
    return false;
}
bool Position::F(){
    //Data Acquisition Complete
    return false;
}