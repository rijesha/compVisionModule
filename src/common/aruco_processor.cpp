#include "aruco_processor.h"
#include <stdlib.h>     /* abs */

ArUcoProcessor::ArUcoProcessor(){}

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize){
    detector = new MarkerDetector();
    detector->setDictionary("ALL_DICTS");
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

    auto markers = detector->detect(image);
    for (auto m : markers){
        if (m.id == markerID){
            correctMarker = &m;
            foundMarkers = true;
        }
    }
 }

int sign(int x) {
    return (x > 0) - (x < 0);
}

Position ArUcoProcessor::calculatePose(){
    Position p;
    if (foundMarkers){
        bool useExtrinsic = true;
        if (((float) (clock() - lastMarkerTime) > RESET_TIME*CLOCKS_PER_SEC) || (abs(eulersAngles[1]) > 55) || (abs(eulersAngles[0]) < 150) || (abs(eulersAngles[3]) > 30) ){
            useExtrinsic = false;
            //cout << "NOT USING EXTRINSIC" << endl;
        }
        lastMarkerTime = clock();
        bool badData = true;
        do {
            rvecs = correctMarker->Rvec;
            tvecs = correctMarker->Tvec;
            //aruco::estimatePoseSingleMarkers(markerCorners, targetSize, camparams.camera_matrix, camparams.dist_coefs, rvecs, tvecs, useExtrinsic);
            if ( tvecs[0][2] > 0){
                badData = false;
            }
            else {
                useExtrinsic = false;
                cout << p.getInfoString();
                cout << "FAILED FAILED FAILED" << endl;
            }
            p = Position(rvecs, tvecs);
            eulersAngles = p.eulersAngles;
        }
        while (badData);
        return p;
    }
    return p;
}


Mat ArUcoProcessor::drawMarkersAndAxis(Mat image, bool alsoDrawAxis){
    Mat out = image.clone();
    try{
        if (foundMarkers){
            CvDrawingUtils::draw3dAxis(out, *correctMarker, camparams);
            //drawDetectedMarkers(out, markerCorners, markerIds);
            //if (alsoDrawAxis){
            //    for(size_t i=0; i<markerIds.size(); i++)
            //        drawAxis(out, camparams.camera_matrix, camparams.dist_coefs, rvecs[i], tvecs[i], targetSize);
            //}
        }
    } catch (...)
    {

    }
    
    return out;
}

Mat ArUcoProcessor::getMarker(int markerNumber,int markerpixels){
    Mat markerImage;
    //drawMarker(dictionary, 17, 600, markerImage, 1);
    cout << "NOT IMPLEMENTED" << endl;
    return markerImage;
}
    