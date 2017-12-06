#include "location_processor.hpp"

LocationProcessor::LocationProcessor(){
    vCap = VideoCapture(DEVICE_ID);
    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);
    //vCap = VideoCapture("v4l2src device=/dev/video1 ! video/x-raw, framerate=30/1, width=640, height=480, format=YUYV ! videoconvert ! appsink");

    CameraParameters camparams;
    camparams.width = CAM_WIDTH;
    camparams.height = CAM_HEIGHT;
    camparams.camera_matrix = CAMPARAMS_CAMERA_MATRIX;
    camparams.dist_coefs = CAMPARAMS_DIST_COEFS;

    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);
}

bool LocationProcessor::LocationProcessingThread(void){
    shutdownFlag = false;
    while (!shutdownFlag){
        processImage();
    }
}

void LocationProcessor::Stop(void){
    shutdownFlag = true;
}

bool LocationProcessor::processImage(void){
    while(true){
        arProc.foundMarkers = false;
        vCap.read(original);
        imageAcquisitionTime = clock();
        if ( !original.data )
        {
            return false;
        }

        arProc.processFrame(original);
        markerDetectionTime = clock();
        arProc.calculatePose();
        posecalculationTime = clock();
        image = arProc.drawMarkersAndAxis(original);
        drawingImageTime = clock();
    }
}

bool LocationProcessor::switchToCloseInProcessing(bool closeIn){
    if (closeIn)
        arProc.changeCornerRefinementWindowSize(15);
    else
        arProc.changeCornerRefinementWindowSize(5);
    
    return true;
}
