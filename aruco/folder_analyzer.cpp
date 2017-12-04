#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "undistort_image.h"
#include "configuration.h"
#include "argparse/argparse.hpp"
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

void processImage(Mat fun){
    arProc.foundMarkers = false;
    arProc.processFrame(fun);
    
    
    if (arProc.markerIds.size() > 1){
        cout << "MULTIPLE MARKERS: " <<  arProc.markerIds.size() << endl;
        cout << arProc.markerIds[0] <<' ' << arProc.markerIds[1] << endl;
        cout << current_file << endl;
        arProc.calculatePose();
        cout << arProc.getInfoString();
        
    }
    
    //
    //image = arProc.drawMarkersAndAxis(original);
}

int main(int argc, const char** argv )
{   
     // make a new ArgumentParser
    ArgumentParser parser;
    parser.addArgument("-i", "--input_folder", 1, false);
    parser.addArgument("-d", "--devices", 1, true);
    parser.addArgument("-o", "--output", 1, true);
    parser.addArgument("-w", "--withoutCornerSubPixel", 1, true);
    parser.addArgument("-s", "--saveData", 1, true);
    parser.addArgument("-t", "--debugTiming", 1, true);
    parser.addArgument("-q", "--quiet", 1, true);
    parser.parse(argc, argv);

    string folderpath = parser.retrieve<string>("i");
    cout << folderpath << endl;

    bool runWithoutSubPixel = false;
    string defaultDP = parser.retrieve<string>("w");
    if (defaultDP.length() != 0)
        runWithoutSubPixel = (stoi(defaultDP) == 1);

    CameraParameters camparams;
    camparams.width = CAM_WIDTH;
    camparams.height = CAM_HEIGHT;
    camparams.camera_matrix = CAMPARAMS_CAMERA_MATRIX;
    camparams.dist_coefs = CAMPARAMS_DIST_COEFS;

    cout << CAMPARAMS_CAMERA_MATRIX <<endl;
    cout << CAMPARAMS_DIST_COEFS << endl;

    ui = UndistortImage(camparams);
    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);

    if (runWithoutSubPixel)
        arProc = ArUcoProcessor(camparams, TARGET_WIDTH, new DetectorParameters);
    

    cout << "Iterating" << endl;
    path p(folderpath);
    directory_iterator end_itr;

    for (directory_iterator itr(p); itr != end_itr; ++itr)
    {
        if (is_regular_file(itr->path())) {
            current_file = itr->path().string();
            Mat fun = imread(current_file, CV_LOAD_IMAGE_COLOR);
            if(! fun.data )                              // Check for invalid input
            {
                cout <<  "Could not open or find the image" << std::endl ;
            }
            processImage(fun);

            /*
            imshow( "Display window", original );
            char c = waitKey(10);
            if (c == 27 || c == 113){
                break;
            }*/
            //cout << current_file << endl;
        }
    }
    

    return 0;
}