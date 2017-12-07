#include "location_processor.hpp"

LocationProcessor::LocationProcessor(){
    vCap = VideoCapture(DEVICE_ID);
    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);
    //vCap = VideoCapture("v4l2src device=/dev/video1 ! video/x-raw, framerate=30/1, width=640, height=480, format=YUYV ! videoconvert ! appsink");
    usleep(100);
    CameraParameters camparams;
    camparams.width = CAM_WIDTH;
    camparams.height = CAM_HEIGHT;
    camparams.camera_matrix = (Mat_<double>(3,3) << CM_0, CM_1, CM_2, CM_3, CM_4, CM_5, CM_6, CM_7, CM_8);
    camparams.dist_coefs = (Mat_<double>(1,5) << DC_0, DC_1, DC_2, DC_3, DC_4);
    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);
}

void LocationProcessor::LocationProcessingThread(void){
    shutdownFlag = false;
    while (!shutdownFlag){
        if (processImage()){
            #ifdef PRINT_INFO_STRING
                cout << arProc.getInfoString();
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
    arProc.calculatePose();
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
