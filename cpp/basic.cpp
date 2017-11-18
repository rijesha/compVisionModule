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
    testFile << "depth, x, y, angle, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3" << endl;

    VideoCapture vCap(deviceID);

    CameraParameters camparams;
    camparams.width = CAMPARAMS_WIDTH;
    camparams.height = CAMPARAMS_HEIGHT;
    camparams.camera_matrix = CAMPARAMS_CAMERA_MATRIX;
    camparams.dist_coefs = CAMPARAMS_DIST_COEFS;

    cout << CAMPARAMS_CAMERA_MATRIX <<endl;
    cout << CAMPARAMS_DIST_COEFS << endl;

    UndistortImage ui(camparams);

    Mat image;
    Mat markerImage;

    ArUcoProcessor arProc(camparams, TARGET_WIDTH);

    if (runWithoutSubPixel)
        arProc = ArUcoProcessor(camparams, TARGET_WIDTH, new DetectorParameters);
        
    imwrite( "marker28.jpg", arProc.getMarker(28));
    vCap.read(image);
    Mat dstImg = image.clone();

    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);
    int i = 0;
    int count = 10;
    
    string userinput;

    clock_t t1,t2;
    t1=clock();
    t2 = clock(); 
    cout << "seconds : " << ((float)t2-(float)t1) / CLOCKS_PER_SEC << endl;
    t1=clock();
    t2=clock(); 

    int sixtyFrames = 0;

    while(1) {
        if (arProc.foundMarkers && saveData && sixtyFrames > 30){
            if (saveData == true || count < 10){
                cout << "Save current Data?" << endl;
                char progress;
                cin >> progress;
                cout << progress << endl;
                if ( !cin.fail() && progress=='y' && progress!='n' ){
                    if (count == 10){
                        userinput.clear();
                        cout << "Current Location input as: depth,x,y,angle" << endl;
                        cin >> userinput;
                        count = 0;
                    }
                    count++;
                    cout << "SAVING measurement : ";
                    cout << count << endl;
                    if (count == 10)
                        cout << "10 Measurements have been saved!!!" << endl; 
                    
                    testFile << userinput << ',';
                    testFile << arProc.tvecs[0][0] << ',' << arProc.tvecs[0][1] << ',' << arProc.tvecs[0][2] << ',';
                    testFile << arProc.eulersAngles[0] << ',' << arProc.eulersAngles[1] << ',' << arProc.eulersAngles[2] << ',';
                    testFile << arProc.worldPos.at<double>(0) << ',' << arProc.worldPos.at<double>(1) << ',' << arProc.worldPos.at<double>(2) << endl;
                }
            }
            sixtyFrames = 0;
        }
        else{
            vCap.read(image);
            if ( !image.data )
            {
                printf("No image data \n");
                return -1;
            }

            arProc.processFrame(image);
            arProc.calculatePose();
            arProc.drawMarkersAndAxis(image);
            
            if (!saveData){
                //cout << arProc.tvecs[0][0] << ',' << arProc.tvecs[0][1] << ',' << arProc.tvecs[0][2]<< ",   ";
                //cout << arProc.eulersAngles[0] << ',' << arProc.eulersAngles[1] << ',' << arProc.eulersAngles[2] << endl;
                cout << arProc.worldPos.at<double>(0) << ',' << arProc.worldPos.at<double>(1) << ',' << arProc.worldPos.at<double>(2) << endl;
            }
            
            sixtyFrames++;

            #ifdef DISPLAY_IMAGE
            namedWindow("Display Image", WINDOW_AUTOSIZE );
            Mat out;
            image = ui.undistortAcquiredImage(image, out);
            imshow("Display Image", image);
            stringstream filename("without_cornesr_refinement/image_no_" + std::to_string(i) + ".jpg");; 
            imwrite( filename.str(), image );
            i++;
            #endif
            
            char c = waitKey(10);
            if (c == 27 || c == 113){
                break;
            }
        }
        
        try {
     
        }
        catch (...){
            cout << "CAUGHT ERROR" << endl;
        }

        
        
    }
    
    testFile.close();
    return 0;
}