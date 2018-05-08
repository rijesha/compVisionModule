#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <common/undistort_image.h>
#include "../configuration.h"
#include <common/cvm_argument_parser.hpp>
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include <common/aruco_processor.h>
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
Position p;

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
    p = arProc.calculatePose();
    posecalculationTime = clock();
    cvtColor(original, image, CV_GRAY2BGR );
    image = arProc.drawMarkersAndAxis(image);
    drawingImageTime = clock();
}

int main(int argc, const char** argv )
{   
    CVMArgumentParser ap(argc, argv, true, false, false, false);

    if (ap.saveData){
        testFile = ofstream("TestData.csv", ofstream::out);
        testFile << "imgnum, count, depth, azimuth, camera azimuth, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3" << endl;
    }
    if (ap.saveTiming){
        timingFile = ofstream("timeData.csv", ofstream::out);
        timingFile << "veryoverallCout, overallCount, foundMarker, savedImage, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime, depth" << endl;
    }

    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(ap.calib_file_path);

    int width = CamParam.CamSize.width;
    int height = CamParam.CamSize.height;

    vCap = VideoCapture("v4l2src device=/dev/video1 ! video/x-raw,format=GRAY8,width=1280,height=960,framerate=30/1 ! videoconvert ! appsink");
    cout << "openeded device" << endl;
    
    ui = UndistortImage(CamParam);
    arProc = ArUcoProcessor(CamParam, TARGET_WIDTH);

    vCap.read(image);

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


    Mat undistorted = image.clone();
    while(1) {
        processImage();
        
        if (ap.saveVideo){
            writer << image;
        }
        
        #ifdef DISPLAY_IMAGE
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        if (!ap.saveTiming){
            ui.undistortAcquiredImage(image, &undistorted);
            imshow("Display Image", undistorted);
        } else {
            imshow("Display Image", image);
        }
        
        #endif

        char c = waitKey(10);
        if (c == 27 || c == 113){
            break;
        }
        
        
        if (!ap.saveData && arProc.foundMarkers && !ap.quiet){
            cout << p.getInfoString();
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
                cout << p.getInfoString();
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
                testFile << p.getInfoString();
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