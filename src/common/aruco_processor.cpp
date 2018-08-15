#include "aruco_processor.h"
#include <stdlib.h> /* abs */

ArUcoProcessor::ArUcoProcessor() {}

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize, UndistortImage ui)
{   
    this->ui = ui;
    detector = new MarkerDetector();
    pose_tracker = new MarkerPoseTracker();
    detector->setDictionary("ARUCO_MIP_16h3");
    MarkerDetector::Params &params = detector->getParameters();

    params.setDetectionMode(DM_FAST, 0.02);
    params.setCornerRefinementMethod(CORNER_LINES);

    //this->dictionary = getPredefinedDictionary(dictionaryName);

    //this->detectorParameters = new aruco::MarkerDetector;
    //detectorParameters->cornerRefinementMethod = CORNER_REFINE_SUBPIX;
    //detectorParameters->cornerRefinementWinSize = 5;
    //detectorParameters->cornerRefinementMinAccuracy = .001;
    //detectorParameters->cornerRefinementMaxIterations = 2000;
    //detectorParameters->adaptiveThreshWinSizeMin = 17;
    //detectorParameters->adaptiveThreshWinSizeMax = 17;

    this->camparams = camparams;
    this->targetSize = targetSize;
    foundMarkers = false;
    lastMarkerTime = clock();
}

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize, Ptr<MarkerDetector> detectorParameters, UndistortImage ui)
{
    this->ui = ui;
    //this->dictionary = getPredefinedDictionary(dictionaryName);
    //this->detectorParameters = detectorParameters;
    pose_tracker = new MarkerPoseTracker();
    this->camparams = camparams;
    this->targetSize = targetSize;
    foundMarkers = false;
    lastMarkerTime = clock();
}

void ArUcoProcessor::changeCornerRefinementWindowSize(int size)
{
    //detectorParameters->cornerRefinementWinSize = size;
}

void ArUcoProcessor::processFrame(Mat image, int markerID)
{
    foundMarkers = false;
    vecsUpdated = false;

    detectedMarkers = detector->detect(image);
    for (Marker m : detectedMarkers)
    {
        if (m.id == markerID)
        {
            foundMarkers = true;
            m.copyTo(detectedMarker);
        }
    }
}

int sign(int x)
{
    return (x > 0) - (x < 0);
}

Position ArUcoProcessor::calculatePose()
{
    Position p;
    if (foundMarkers)
    {   
        cout << "really starting" << endl;
        if (positiveYaw()) {

            float test1[] = {2.9024532, 0, -1.2022355};
            Mat p_rvec(1,3,CV_32F,test1);

            float test2[] = {1, 1, 1};
            Mat p_tvec(1,3,CV_32F,test2);

            cout << p_rvec.type() << CV_32F << endl;
            cout << p_rvec << endl;
            pose_tracker->estimatePoseWithHint(detectedMarker, camparams, this->targetSize, p_rvec, p_tvec);
            cout << "Positive YAW ";

        } else {
            float test1[] = {2.9024532, 0, 1.2022355};
            Mat n_rvec(1,3,CV_32F,test1);

            float test2[] = {1, 1, 1};
            Mat n_tvec(1,3,CV_32F,test2);

            cout << n_rvec.type() << CV_32F << endl;
            pose_tracker->estimatePoseWithHint(detectedMarker, camparams, this->targetSize, n_rvec, n_tvec);
            cout << "Negative YAW ";
        }
        cout << "really ending" << endl;
        Mat RTmatrix = pose_tracker->getRTMatrix();
        cout << "RVEC  " << endl;
        cout << pose_tracker->getRvec() << endl;
        cout << "done RVEC" << endl;
        if (!RTmatrix.empty())
        {
            p = Position(RTmatrix);
            eulersAngles = p.eulersAngles;
        }
        cout << p.azi << endl;
        return p;
    }
    return p;
}

bool ArUcoProcessor::positiveYaw()
{
    Marker m = ui.undistortMarkerPoints(detectedMarker);
    Point_<float> left_pt  = m[0] - m[3];
    Point_<float> right_pt = m[1] - m[2];
    double left = sqrt(pow(left_pt.x,2) + pow(left_pt.y,2));
    double right = sqrt(pow(right_pt.x,2) + pow(right_pt.y,2));
    return (right - left) > 0;
}

Mat ArUcoProcessor::drawMarkersAndAxis(Mat image, bool alsoDrawAxis)
{
    Mat out = image.clone();

    try
    {
        if (foundMarkers)
        {
            aruco::CvDrawingUtils::draw3dAxis(out, detectedMarker, camparams);
            aruco::CvDrawingUtils::draw3dCube(out, detectedMarker, camparams);
        }
    }
    catch (...)
    {
    }

    return out;
}
