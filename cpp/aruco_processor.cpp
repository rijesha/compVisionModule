#include "aruco_processor.h"


ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize, PREDEFINED_DICTIONARY_NAME dictionaryName){
    this->dictionary = getPredefinedDictionary(dictionaryName);

    this->detectorParameters = new aruco::DetectorParameters;
    detectorParameters->cornerRefinementMethod = CORNER_REFINE_SUBPIX;
    detectorParameters->cornerRefinementWinSize = 15;  
    detectorParameters->cornerRefinementMinAccuracy = .01;
    detectorParameters->cornerRefinementMaxIterations = 200;

    this->camparams = camparams;
    this->targetSize = targetSize;
    foundMarkers = false;
}

ArUcoProcessor::ArUcoProcessor(CameraParameters camparams, float targetSize, Ptr<DetectorParameters> detectorParameters, PREDEFINED_DICTIONARY_NAME dictionaryName){
    this->dictionary = getPredefinedDictionary(dictionaryName);
    this->detectorParameters = detectorParameters;
    this->camparams = camparams;
    this->targetSize = targetSize;
    foundMarkers = false;
}

void ArUcoProcessor::processFrame(Mat image){
    foundMarkers = false;
    vecsUpdated = false;

    detectMarkers(image, dictionary, markerCorners, markerIds, detectorParameters);
    foundMarkers = (markerIds.size() > 0);
}

void ArUcoProcessor::calculatePose(){
    if (foundMarkers){
        aruco::estimatePoseSingleMarkers(markerCorners, targetSize, camparams.camera_matrix, camparams.dist_coefs, rvecs, tvecs);
        Rodrigues(rvecs[0],rotMat);
        calcEulerAngles();

        Mat invRotMat;
        transpose(rotMat, invRotMat);
        worldPos = -invRotMat * Mat(tvecs[0]);
    }
}

void ArUcoProcessor::calcEulerAngles(){

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

void ArUcoProcessor::drawMarkersAndAxis(Mat image, bool alsoDrawAxis){
    if (foundMarkers){
        drawDetectedMarkers(image, markerCorners, markerIds);
        if (alsoDrawAxis){
            for(size_t i=0; i<markerIds.size(); i++)
                drawAxis(image, camparams.camera_matrix, camparams.dist_coefs, rvecs[i], tvecs[i], targetSize);
        }
    }
}


Mat ArUcoProcessor::getMarker(int markerNumber,int markerpixels){
    Mat markerImage;
    drawMarker(dictionary, 17, 600, markerImage, 1);
    return markerImage;
}

    