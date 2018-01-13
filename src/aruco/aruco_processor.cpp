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
            Position p(rvecs, tvecs);
            eulersAngles = p.eulersAngles;
        }
        while (badData);
        return p;
    }
    return p;
}

Position::Position(){
    emptyPosition = true;
}
Position::Position(vector< Vec3d > rvecs, vector< Vec3d > tvecs) : rvecs(rvecs), tvecs(tvecs){
    emptyPosition = false;
    Rodrigues(rvecs[0],rotMat);
    calcEulerAngles();
    Mat invRotMat;
    transpose(rotMat, invRotMat);
    worldPos = -invRotMat * Mat(tvecs[0]);
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

    output << tvecs[0][0] << ',' << tvecs[0][1] << ',' << tvecs[0][2] << ',';
    output << eulersAngles[0] << ',' << eulersAngles[1] << ',' << eulersAngles[2] << ',';
    output << worldPos.at<double>(0) << ',' << worldPos.at<double>(1) << ',' << worldPos.at<double>(2) << endl;
    return output.str();
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

    