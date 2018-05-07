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
    x = _t[0];
    y = _t[1];
    depth = _t[2];
    
    float* _eA = eulersAngles.ptr<float>();
    ele  = _eA[0];
    azi  = _eA[1];
    tilt = _eA[2];
}

Position::Position(Mat rvecs, Mat tvecs) : rvecs(rvecs), tvecs(tvecs){
    creation_time = clock();
    time_since_last_positon = creation_time - last_creation_time;
    last_creation_time = creation_time;
    emptyPosition = false;
    Rodrigues(rvecs,rotMat);
    
    calcEulerAngles();
    cout << "you didn't seg fault 25" << endl;
    Mat invRotMat;
    transpose(rotMat, invRotMat);
    cout << "you didn't seg fault 32" << endl;
    cout << invRotMat << endl;
    cout << invRotMat.size() << endl;
    cout << (-invRotMat) << endl;
    cout << (-invRotMat).size() << endl;
    cout << tvecs.size() << endl;
    cout << invRotMat.cols << " " << tvecs.rows << endl;
    
    worldPos = -rotMat.inv() * tvecs.t();
    RTMatrix = worldPos;
    cout << "you didn't seg fault 31" << endl;
    float* _t = tvecs.ptr<float>();
    x = _t[0];
    y = _t[1];
    depth = _t[2];
    
    float* _eA = eulersAngles.ptr<float>();
    ele  = _eA[0];
    azi  = _eA[1];
    tilt = _eA[2];
}

void Position::calcEulerAngles(){
    
    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;

    float* _r = rotMat.ptr<float>();
    float projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};
    //cout << _r[0] << _r[1] << _r[2] << endl << _r[3] << _r[4] << _r[5] << endl << _r[6] << _r[7] << _r[8]  << endl;

    Mat test = Mat(3,4,CV_32FC1,projMatrix);
    cout << test << endl;
    cout << "about to seg fault" << endl;
    decomposeProjectionMatrix( test,
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulersAngles);

    cout << "you didn't seg fault" << endl;
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