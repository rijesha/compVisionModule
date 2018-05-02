#ifndef ARUCO_PROCESSOR_H
#define ARUCO_PROCESSOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../libs/aruco/include/aruco/aruco.h"
#include "undistort_image.h"
//#include "../../configuration.h"
#include "position.hpp"
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
    Ptr<MarkerDetector> detector;
    bool vecsUpdated;
    CameraParameters camparams;
    float targetSize;
    clock_t lastMarkerTime;
public:
    ArUcoProcessor();
    ArUcoProcessor(CameraParameters camparams, float targetSize);
    ArUcoProcessor(CameraParameters camparams, float targetSize, Ptr<MarkerDetector> detectorParameters);

    void changeCornerRefinementWindowSize(int);
    void processFrame(Mat image, int markerID = 19);
    Position calculatePose();
    Mat drawMarkersAndAxis(Mat image, bool drawAxis = true);
    Mat getMarker(int markerNumber = 19,int markerpixels = 600);

    vector< int > markerIds;
    vector< vector<Point2f> > rejectedCandidates;
    vector< Vec3d > rvecs, tvecs;
    Vec3d eulersAngles;
    bool foundMarkers;

    Ptr<Marker> correctMarker;
};

#endif /* ARUCO_PROCESSOR_H */
