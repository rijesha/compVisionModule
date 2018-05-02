#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "undistort_image.h"
#include "../configuration.h"
#include "../cvm_argument_parser.hpp"
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include "aruco_processor.h"
#include <unistd.h>
#include "boost/filesystem.hpp"
using namespace boost::filesystem;

using namespace cv;

ofstream testFile, timingFile;
string testFileFolder;

clock_t imageAcquisitionTime, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime;

VideoCapture vCap;
UndistortImage ui;
Mat image;
Mat original;

ArUcoProcessor arProc;
int multipleMarkerCount = 0;
string current_file;
Position p;

void processImage(Mat fun){
    arProc.foundMarkers = false;
    arProc.processFrame(fun);
    
    if (arProc.markerIds.size() > 1){
        cout << "MULTIPLE MARKERS: " <<  arProc.markerIds.size() << endl;
        cout << arProc.markerIds[0] <<' ' << arProc.markerIds[1] << endl;
        cout << current_file << endl;
        p = arProc.calculatePose();
        cout << p.getInfoString();
    }
    
    //
    Position p = arProc.calculatePose();
    image = arProc.drawMarkersAndAxis(fun);
}

int main(int argc, const char** argv )
{   
    CVMArgumentParser ap(argc, argv, true, false, true, false);
    
    FileStorage fs(ap.calib_file_path, FileStorage::READ);
    Mat cameraMatrix2, distCoeffs2; 
    int width, height;
    fs["cameraMatrix"] >> cameraMatrix2;
    fs["distCoeffs"] >> distCoeffs2;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    

    CameraParameters camparams;

    camparams.width = width;
    camparams.height = height;
    camparams.camera_matrix = cameraMatrix2;
    camparams.dist_coefs = distCoeffs2;

    cout << camparams.camera_matrix <<endl;
    cout << camparams.dist_coefs << endl;

    ui = UndistortImage(camparams);
    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);

    cout << "Iterating" << endl;
    path p(ap.inputpath);
    directory_iterator end_itr;

    for (directory_iterator itr(p); itr != end_itr; ++itr)
    {
        if (is_regular_file(itr->path())) {
            current_file = itr->path().string();
            Mat fun = imread(current_file, CV_LOAD_IMAGE_COLOR);
            cout << current_file << endl;
            if(! fun.data )                              // Check for invalid input
            {
                cout <<  "Could not open or find the image" << std::endl ;
            }
            processImage(fun);

            
            imshow( "Display window", fun);
            imshow( "Display window2", image);
            char c = waitKey(10000);
            if (c == 27 || c == 113){
                break;
            }
            //cout << current_file << endl;
        }
    }
    

    return 0;
}