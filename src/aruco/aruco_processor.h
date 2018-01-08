#ifndef ARUCO_PROCESSOR_H
#define ARUCO_PROCESSOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "undistort_image.h"
#include "../configuration.h"
#include <ctime>
#include <chrono>

using namespace std;
using namespace cv;
using namespace aruco;

#define RESET_TIME 1 //seconds

class ArUcoProcessor
{
private:
    Ptr<Dictionary> dictionary;
    Ptr<DetectorParameters> detectorParameters;
    bool vecsUpdated;
    CameraParameters camparams;
    float targetSize;
    void calcEulerAngles();
    clock_t lastMarkerTime;
public:
    ArUcoProcessor();
    ArUcoProcessor(CameraParameters camparams, float targetSize, PREDEFINED_DICTIONARY_NAME dictionaryName = DEFAULT_DICTIONARY_NAME);
    ArUcoProcessor(CameraParameters camparams, float targetSize, Ptr<DetectorParameters> detectorParameters, PREDEFINED_DICTIONARY_NAME dictionaryName = DEFAULT_DICTIONARY_NAME);

    void changeCornerRefinementWindowSize(int);
    void processFrame(Mat image);
    void calculatePose();
    Mat drawMarkersAndAxis(Mat image, bool drawAxis = true);
    Mat getMarker(int markerNumber = 19,int markerpixels = 600);

    string getInfoString();

    vector< int > markerIds;
    vector< vector<Point2f> > markerCorners, rejectedCandidates;
    vector< Vec3d > rvecs, tvecs;
    Vec3d eulersAngles;
    Mat rotMat;
    Mat worldPos;
    bool foundMarkers;
};

#endif /* ARUCO_PROCESSOR_H */