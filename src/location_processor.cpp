#include "location_processor.hpp"

LocationProcessor::LocationProcessor(string calib_file_path, int device_id){
        
    FileStorage fs(calib_file_path, FileStorage::READ);
    Mat cameraMatrix2, distCoeffs2; 
    int width, height;
    fs["camera_matrix"] >> cameraMatrix2;
    fs["distortion_coefficients"] >> distCoeffs2;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    
    vCap = VideoCapture(device_id);
    vCap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,height);
    
    CameraParameters camparams;

    camparams.width = width;
    camparams.height = height;
    camparams.camera_matrix = cameraMatrix2;
    camparams.dist_coefs = distCoeffs2;

    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);
}

void LocationProcessor::LocationProcessingThread(void){
    shutdownFlag = false;
    while (!shutdownFlag){
        if (processImage()){
            #ifdef PRINT_INFO_STRING
                //cout << p.getInfoString();
            #endif
        }
    }
}

void LocationProcessor::Stop(void){
    shutdownFlag = true;
    cout << "SETTING SHUTDOWN FLAG: " << shutdownFlag << endl;
}

bool LocationProcessor::processImage(void){
    arProc.foundMarkers = false;
    vCap.read(original);
    imageAcquisitionTime = clock();
    if ( !original.data )
    {
        return false;
    }
    arProc.processFrame(original);
    markerDetectionTime = clock();
    Position p = arProc.calculatePose();
    posecalculationTime = clock();
    image = arProc.drawMarkersAndAxis(original);
    drawingImageTime = clock();
    return arProc.foundMarkers;
}

bool LocationProcessor::switchToCloseInProcessing(bool closeIn){
    if (closeIn)
        arProc.changeCornerRefinementWindowSize(15);
    else
        arProc.changeCornerRefinementWindowSize(5);
    
    return true;
}
