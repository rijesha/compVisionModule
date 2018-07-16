#include "location_processor.hpp"

LocationProcessor::LocationProcessor(string calib_file_path, string device_id){
        
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(calib_file_path);

    width = CamParam.CamSize.width;
    height = CamParam.CamSize.height;
    cout << width << "x" << height << endl;
    
    camera = new Camera(device_id, width, height, true, 10, 20);
    Image image1 = camera->captureFrame();
    cout << "opened device" << endl;
    
    arProc = ArUcoProcessor(CamParam, TARGET_WIDTH);

}

Position LocationProcessor::processImage(void){
    arProc.foundMarkers = false;
    
    Image image1 = camera->captureFrame();
    original = Mat(height, width, CV_8UC1, image1.data);

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
