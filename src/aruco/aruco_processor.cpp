#include "aruco_processor.h"
#include <stdlib.h>     /* abs */

ArUcoProcessor::ArUcoProcessor(){}

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize, PREDEFINED_DICTIONARY_NAME dictionaryName){
    this->dictionary = getPredefinedDictionary(dictionaryName);

    this->detectorParameters = new aruco::DetectorParameters;
    detectorParameters->cornerRefinementMethod = CORNER_REFINE_SUBPIX;
    detectorParameters->cornerRefinementWinSize = 5; 
    detectorParameters->cornerRefinementMinAccuracy = .001;
    detectorParameters->cornerRefinementMaxIterations = 2000;
    detectorParameters->adaptiveThreshWinSizeMin = 17;
    detectorParameters->adaptiveThreshWinSizeMax = 17;

    this->camparams = camparams;
    this->targetSize = targetSize;
    foundMarkers = false;
    lastMarkerTime = clock();
}

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize, Ptr<DetectorParameters> detectorParameters, PREDEFINED_DICTIONARY_NAME dictionaryName){
    this->dictionary = getPredefinedDictionary(dictionaryName);
    this->detectorParameters = detectorParameters;
    this->camparams = camparams;
    this->targetSize = targetSize;
    foundMarkers = false;
    lastMarkerTime = clock();
}

void ArUcoProcessor::changeCornerRefinementWindowSize(int size){
    detectorParameters->cornerRefinementWinSize = size; 
}

void ArUcoProcessor::processFrame(Mat image){
    foundMarkers = false;
    vecsUpdated = false;

    detectMarkers(image, dictionary, markerCorners, markerIds, detectorParameters);
    if (markerIds.size() > 0){
        vector<Point2f> correctMarker;
        for (uint i = 0; i < markerIds.size(); i++){
            if (markerIds[i] == MARKER_ID){
                correctMarker = markerCorners[i];
                markerCorners.clear();
                markerCorners.push_back(correctMarker);
                foundMarkers = true;
            }
        }
        markerIds.clear();
        markerIds.push_back(MARKER_ID);
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
            aruco::estimatePoseSingleMarkers(markerCorners, targetSize, camparams.camera_matrix, camparams.dist_coefs, rvecs, tvecs, useExtrinsic);
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
            drawDetectedMarkers(out, markerCorners, markerIds);
            if (alsoDrawAxis){
                for(size_t i=0; i<markerIds.size(); i++)
                    drawAxis(out, camparams.camera_matrix, camparams.dist_coefs, rvecs[i], tvecs[i], targetSize);
            }
        }
    } catch (...)
    {

    }
    
    return out;
}

Mat ArUcoProcessor::getMarker(int markerNumber,int markerpixels){
    Mat markerImage;
    drawMarker(dictionary, 17, 600, markerImage, 1);
    return markerImage;
}
    