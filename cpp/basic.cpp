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

ofstream testFile, timingFile;
string testFileFolder;

clock_t imageAcquisitionTime, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime;

VideoCapture vCap;
UndistortImage ui;
Mat image;
Mat original;

ArUcoProcessor arProc;

void processImage(){
    arProc.foundMarkers = false;
    vCap.read(original);
    imageAcquisitionTime = clock();
    if ( !original.data )
    {
        printf("No image data \n");
    }

    arProc.processFrame(original);
    markerDetectionTime = clock();
    arProc.calculatePose();
    posecalculationTime = clock();
    image = arProc.drawMarkersAndAxis(original);
    drawingImageTime = clock();
}

int main(int argc, const char** argv )
{   
     // make a new ArgumentParser
    ArgumentParser parser;
    parser.addArgument("-d", "--devices", 1, true);
    parser.addArgument("-o", "--output", 1, true);
    parser.addArgument("-w", "--withoutCornerSubPixel", 1, true);
    parser.addArgument("-s", "--saveData", 1, true);
    parser.addArgument("-t", "--debugTiming", 1, true);
    parser.addArgument("-q", "--quiet", 1, true);
    parser.parse(argc, argv);

    bool runWithoutSubPixel = false;
    string defaultDP = parser.retrieve<string>("w");
    if (defaultDP.length() != 0)
        runWithoutSubPixel = (stoi(defaultDP) == 1);

    int deviceID = 1;
    string devicestring = parser.retrieve<string>("d");
    if (devicestring.length() != 0)
        deviceID =stoi(devicestring);
    
    bool saveData = false;
    string saveDatastr = parser.retrieve<string>("s");
    if (saveDatastr.length() != 0)
        saveData =(stoi(saveDatastr) == 1);
    
    bool saveTiming = false;
    string saveTimingstr = parser.retrieve<string>("t");
    if (saveTimingstr.length() != 0)
        saveTiming =(stoi(saveTimingstr) == 1);

    bool quiet = false;
    string quietstr = parser.retrieve<string>("q");
    if (quietstr.length() != 0)
        quiet =(stoi(quietstr) == 1);


    if (saveData){
        testFile = ofstream("TestData.csv", ofstream::out);
        testFile << "imgnum, count, depth, x, y, azimuth, elevation, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3" << endl;
    }
    if (saveTiming){
        timingFile = ofstream("timeData.csv", ofstream::out);
        timingFile << "veryoverallCout, overallCount, foundMarker, savedImage, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime" << endl;
    }
    

    vCap = VideoCapture(deviceID);
    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);


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
        
    //imwrite( "marker28.jpg", arProc.getMarker(28));
    vCap.read(image);
    Mat dstImg = image.clone();

    string userinput;
    int count = 100;
    int smallcount = 0;
    bool targetReady = false;
    int clearBuffer = 0;
    int overallCount = 1410;
    int veryoverallCout = 0;

    while(1) {
        processImage();

        #ifdef DISPLAY_IMAGE
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        if (!saveTiming){
            Mat out;
            image = ui.undistortAcquiredImage(image, out);
        }
        imshow("Display Image", image);
        #endif

        char c = waitKey(10);
        if (c == 27 || c == 113){
            break;
        }
        if (!saveData && arProc.foundMarkers && !quiet){
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
                cout << arProc.getInfoString();
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
                savingImageTime = clock();
                if (smallcount == 10) {
                    targetReady = false;
                }
            }
        }

        if (saveTiming ){
            veryoverallCout++;
            if (!saveData){
                imwrite( "testData/timing/" + to_string(veryoverallCout) + ".jpg", original );
                savingImageTime = clock();
            }
            float mdTime = markerDetectionTime - imageAcquisitionTime;
            float pcTime = posecalculationTime - markerDetectionTime ;
            float diTime = drawingImageTime - posecalculationTime;
            float siTime = savingImageTime - drawingImageTime;
            
            stringstream timingData;
            timingData << arProc.foundMarkers << ','  << (arProc.foundMarkers && saveData)  << ',' << mdTime /CLOCKS_PER_SEC  << ',' ;
            timingData << pcTime /CLOCKS_PER_SEC << ',' <<  diTime/CLOCKS_PER_SEC << ',' << siTime/CLOCKS_PER_SEC << endl;
            cout << timingData.str();
            timingFile << veryoverallCout << ',' << overallCount << ',' << timingData.str();
        }
    }
    
    testFile.close();
    return 0;
}