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

using namespace cv;

ofstream testFile;
string testFileFolder;

VideoCapture vCap;
UndistortImage ui;
Mat image;
Mat original;

ArUcoProcessor arProc;

void processImage(){
    arProc.foundMarkers = false;
    vCap.read(original);
    if ( !original.data )
    {
        printf("No image data \n");
    }

    arProc.processFrame(original);
    arProc.calculatePose();
    image = arProc.drawMarkersAndAxis(original);
}

int main(int argc, const char** argv )
{   
     // make a new ArgumentParser
    ArgumentParser parser;
    parser.addArgument("-d", "--devices", 1, true);
    parser.addArgument("-o", "--output", 1, true);
    parser.addArgument("-w", "--withoutCornerSubPixel", 1, true);
    parser.addArgument("-s", "--saveData", 1, true);
    parser.parse(argc, argv);

    bool runWithoutSubPixel = false;
    string defaultDP = parser.retrieve<string>("w");
    if (defaultDP.length() != 0)
        runWithoutSubPixel = (stoi( parser.retrieve<string>("w")) == 1);

    int deviceID = 1;
    string devicestring = parser.retrieve<string>("d");
    if (devicestring.length() != 0)
        deviceID =stoi( parser.retrieve<string>("d"));
    
    bool saveData = false;
    string saveDatastr = parser.retrieve<string>("s");
    if (saveDatastr.length() != 0)
        saveData =(stoi( parser.retrieve<string>("s")) == 1);

    testFileFolder = parser.retrieve<string>("output");
    testFile = ofstream("TestData.csv", ofstream::out);
    testFile << "imgnum, count, depth, x, y, azimuth, elevation, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3" << endl;

    vCap = VideoCapture(deviceID);

    CameraParameters camparams;
    camparams.width = CAMPARAMS_WIDTH;
    camparams.height = CAMPARAMS_HEIGHT;
    camparams.camera_matrix = CAMPARAMS_CAMERA_MATRIX;
    camparams.dist_coefs = CAMPARAMS_DIST_COEFS;

    cout << CAMPARAMS_CAMERA_MATRIX <<endl;
    cout << CAMPARAMS_DIST_COEFS << endl;

    ui = UndistortImage(camparams);
    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);

    if (runWithoutSubPixel)
        arProc = ArUcoProcessor(camparams, TARGET_WIDTH, new DetectorParameters);
        
    //imwrite( "marker28.jpg", arProc.getMarker(28));
    vCap.read(image);
    Mat dstImg = image.clone();

    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);
    
/*
    clock_t t1,t2;
    t1=clock();
    t2 = clock(); 
    cout << "seconds : " << ((float)t2-(float)t1) / CLOCKS_PER_SEC << endl;
    t1=clock();
    t2=clock(); 
*/
    string userinput;
    int count = 100;
    int smallcount = 0;
    bool targetReady = false;
    int clearBuffer = 0;
    int overallCount = 0;

    while(1) {
        processImage();

        #ifdef DISPLAY_IMAGE
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        Mat out;
        image = ui.undistortAcquiredImage(image, out);
        imshow("Display Image", image);
        #endif

        char c = waitKey(10);
        if (c == 27 || c == 113){
            break;
        }
        if (!saveData && arProc.foundMarkers){
            cout << arProc.getInfoString();
        }

        if (saveData && arProc.foundMarkers){
            if (count == 100){
                userinput.clear();
                cout << "Current Location input as: depth,x,y,azimuth,elevation" << endl;
                cin >> userinput;
                count = 0;
            }

            clearBuffer++;

            if (!targetReady) {
                cout << "Save current Data? (y/n) or (u) to reinput userdata" << endl;
                char progress;
                cin >> progress;
                cout << progress << endl;
                if ( !cin.fail() && progress=='y' && progress!='n' ){
                    targetReady = true;
                } else if(progress=='u'){
                    count = 100;
                }
                
                smallcount = 0;
                clearBuffer = 0;
            }

            if (targetReady && smallcount < 10 && clearBuffer > 10) {
                smallcount++;
                count++;
                overallCount++;
                testFile << overallCount << ',' << count << ',' << userinput << ',';
                testFile << arProc.getInfoString();
                cout << "SAVING measurement : " << count << endl;
                imwrite( "testData/raw/" + to_string(overallCount) + ".jpg", original );
                imwrite( "testData/undistorted/" + to_string(overallCount) + ".jpg", image );
                if (smallcount == 10) {
                    targetReady = false;
                }
            }
        }
    }
    
    testFile.close();
    return 0;
}