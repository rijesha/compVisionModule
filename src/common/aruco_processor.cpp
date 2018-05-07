#include "aruco_processor.h"
#include <stdlib.h>     /* abs */

ArUcoProcessor::ArUcoProcessor(){}

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize){
    detector = new MarkerDetector();
    pose_tracker = new MarkerPoseTracker();
    detector->setDictionary("ARUCO_MIP_16h3");
    MarkerDetector::Params &params= detector->getParameters();
    
    params.setDetectionMode(DM_NORMAL, 0);
    params.setCornerRefinementMethod(CORNER_SUBPIX);
    
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

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize, Ptr<MarkerDetector> detectorParameters){
    //this->dictionary = getPredefinedDictionary(dictionaryName);
    //this->detectorParameters = detectorParameters;
    pose_tracker = new MarkerPoseTracker();
    this->camparams = camparams;
    this->targetSize = targetSize;
    foundMarkers = false;
    lastMarkerTime = clock();
}

void ArUcoProcessor::changeCornerRefinementWindowSize(int size){
    //detectorParameters->cornerRefinementWinSize = size; 
}

void ArUcoProcessor::processFrame(Mat image ,int markerID){
    foundMarkers = false;
    vecsUpdated = false;

    detectedMarkers = detector->detect(image);
    for (Marker m : detectedMarkers){
        if (m.id == markerID){
            foundMarkers = true;
            m.copyTo(detectedMarker);
        }
    }
 }

int sign(int x) {
    return (x > 0) - (x < 0);
}

Position ArUcoProcessor::calculatePose(){
    Position p;
    if (foundMarkers){
        pose_tracker->estimatePose(detectedMarker,camparams, 0.12);
        Mat RTmatrix = pose_tracker->getRTMatrix();
        if (!RTmatrix.empty()){
            cout << "new data" << endl;
            cout << pose_tracker->getRTMatrix() << endl;
            cout << pose_tracker->getRvec() << endl;
            cout << detectedMarker.Rvec << endl;
            cout << pose_tracker->getTvec() << endl;
            cout << detectedMarker.Tvec << endl;

            Position tempP(detectedMarker.Rvec, detectedMarker.Tvec);
            cout << "didn't seg fault yet 98" << endl;
            p = tempP;
            
            eulersAngles = p.eulersAngles;
            cout << "didn't seg fault yet 85" << endl;
            p = Position(pose_tracker->getRTMatrix());      
        }
        return p;

    }
    return p;
}


Mat ArUcoProcessor::drawMarkersAndAxis(Mat image, bool alsoDrawAxis){
    Mat out = image.clone();
    
    try{
        if (foundMarkers){
            aruco::CvDrawingUtils::draw3dAxis(out, detectedMarker, camparams);
            aruco::CvDrawingUtils::draw3dCube(out, detectedMarker, camparams);
        }
    } catch (...)
    {

    }
    
    return out;
}

   