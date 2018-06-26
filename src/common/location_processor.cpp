#include "location_processor.hpp"

LocationProcessor::LocationProcessor(string calib_file_path, string device_id){
        
    FileStorage fs(calib_file_path, FileStorage::READ);
    Mat cameraMatrix2, distCoeffs2; 
    fs["camera_matrix"] >> cameraMatrix2;
    fs["distortion_coefficients"] >> distCoeffs2;
    fs["image_width"] >> width;
    fs["image_height"] >> height;

    camera = Camera(device_id, width, height);
    CameraParameters camparams(cameraMatrix2, distCoeffs2, Size(width, height));

    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);
}

Position LocationProcessor::processImage(void){
    arProc.foundMarkers = false;
    Image image1 = camera.captureFrame();
    original = Mat(height, width, CV_8UC3, image1.data);
    imageAcquisitionTime = clock();
    if ( !original.data )
    {
        return Position();
    }
    arProc.processFrame(original);
    markerDetectionTime = clock();
    Position p = arProc.calculatePose();
    posecalculationTime = clock();
    return p;
}
