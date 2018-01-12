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

using namespace cv;

ofstream testFile, timingFile;
string testFileFolder;

clock_t imageAcquisitionTime, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime;

VideoCapture vCap;
VideoWriter writer;
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
    CVMArgumentParser ap(argc, argv, true, false, false);

    if (ap.saveData){
        testFile = ofstream("TestData.csv", ofstream::out);
        testFile << "imgnum, count, depth, azimuth, camera azimuth, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3" << endl;
    }
    if (ap.saveTiming){
        timingFile = ofstream("timeData.csv", ofstream::out);
        timingFile << "veryoverallCout, overallCount, foundMarker, savedImage, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime, depth" << endl;
    }

    FileStorage fs(ap.calib_file_path, FileStorage::READ);
    Mat cameraMatrix2, distCoeffs2; 
    int width, height;
    fs["camera_matrix"] >> cameraMatrix2;
    fs["distortion_coefficients"] >> distCoeffs2;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    
    cout << width;
    cout << height;

    vCap = VideoCapture(ap.deviceID);
    vCap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,height);
    //vCap = VideoCapture("v4l2src device=/dev/video1 ! video/x-raw, framerate=30/1, width=640, height=480, format=YUYV ! videoconvert ! appsink");

    CameraParameters camparams;

    camparams.width = width;
    camparams.height = height;
    camparams.camera_matrix = cameraMatrix2;
    camparams.dist_coefs = distCoeffs2;

    cout << camparams.camera_matrix <<endl;
    cout << camparams.dist_coefs << endl;
    cout << cameraMatrix2 <<endl;
    cout << distCoeffs2 << endl;

    ui = UndistortImage(camparams);
    arProc = ArUcoProcessor(camparams, TARGET_WIDTH);

    //imwrite( "marker28.jpg", arProc.getMarker(28));
    vCap.read(image);
    Mat dstImg = image.clone();

    if (ap.saveVideo){
        writer = cv::VideoWriter("out.avi", VideoWriter::fourcc('M','J','P','G'), 24, image.size());
    }

    string userinput;
    int count = 100;
    int smallcount = 0;
    bool targetReady = false;
    int clearBuffer = 0;
    int overallCount = 10900;
    int veryoverallCout = 0;

    while(1) {
        processImage();
        if (ap.saveVideo){
            writer << image;
        }

        #ifdef DISPLAY_IMAGE
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        if (!ap.saveTiming){
            Mat out;
            image = ui.undistortAcquiredImage(image, out);
        }
        imshow("Display Image", image);
        #endif

        char c = waitKey(10);
        if (c == 27 || c == 113){
            break;
        }
        if (!ap.saveData && arProc.foundMarkers && !ap.quiet){
            cout << arProc.getInfoString();
        }

        if (ap.saveData && arProc.foundMarkers){
            if (count == 100){
                testFile.flush();
                userinput.clear();
                cout << "Current Location input as: depth,azimuth, cameraazimuth" << endl;
                cin >> userinput;
                count = 0;
            }

            clearBuffer++;

            if (!targetReady) {
                cout << arProc.getInfoString();
                cout << "Save current Data? (y/n) or (u) to reinput userdata" << endl;
                char progress;
                int prog = cin.get();
                //cin >> progress;
                cout << prog << endl;
                if ( !cin.fail() && (prog=='y' || prog=='\n')&& prog!='n' ){
                    targetReady = true;
                } else if(prog=='u'){
                    count = 100;
                }
                if (prog!='\n'){
                    cin.get();
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

        if (ap.saveTiming && arProc.foundMarkers){
            veryoverallCout++;
            if (!ap.saveData){
                imwrite( "testData/timing/" + to_string(veryoverallCout) + ".jpg", original );
                savingImageTime = clock();
            }
            float mdTime = markerDetectionTime - imageAcquisitionTime;
            float pcTime = posecalculationTime - markerDetectionTime ;
            float diTime = drawingImageTime - posecalculationTime;
            float siTime = savingImageTime - drawingImageTime;
            
            stringstream timingData;
            timingData << arProc.foundMarkers << ','  << (arProc.foundMarkers && ap.saveData)  << ',' << mdTime /CLOCKS_PER_SEC  << ',' ;
            timingData << pcTime /CLOCKS_PER_SEC << ',' <<  diTime/CLOCKS_PER_SEC << ',' << siTime/CLOCKS_PER_SEC << ',' << arProc.tvecs[0][2] << endl;
            cout << timingData.str();
            timingFile << veryoverallCout << ',' << overallCount << ',' << timingData.str();
        }
    }
    
    testFile.close();
    writer.release();
    return 0;
}