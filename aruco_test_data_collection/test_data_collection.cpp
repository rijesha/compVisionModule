#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <common/undistort_image.h>
#include <common/configuration.h>
#include <common/cvm_argument_parser.hpp>
#include <ctime>
#include <chrono>
#include <cstring>
#include <iostream>
#include <fstream>
#include <time.h>
#include <common/aruco_processor.h>
#include <unistd.h>
#include <camera-v4l2/camera.h>

using namespace cv;
using namespace std;

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
Camera *camera;

int width;
int height;

void processImage()
{
    arProc.foundMarkers = false;
    Image image1 = camera->captureFrame();
    original = Mat(height, width, CV_8UC1, image1.data);

    imageAcquisitionTime = clock();
    if (!original.data)
    {
        printf("No image data \n");
    }

    arProc.processFrame(original);
    markerDetectionTime = clock();
    p = arProc.calculatePose();
    posecalculationTime = clock();
    cvtColor(original, image, cv::COLOR_GRAY2BGR);
    image = arProc.drawMarkersAndAxis(image);
    drawingImageTime = clock();
}

int main(int argc, const char **argv)
{
    CVMArgumentParser ap(argc, argv, true, false, false, false);

    if (ap.saveData)
    {
        testFile = ofstream("TestData.csv", ofstream::out);
        testFile << "imgnum, count, depth, azimuth, camera azimuth, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3" << endl;
    }
    if (ap.saveTiming)
    {
        timingFile = ofstream("timeData.csv", ofstream::out);
        timingFile << "veryoverallCout, overallCount, foundMarker, savedImage, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime, depth" << endl;
    }

    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(ap.calib_file_path);

    width = CamParam.CamSize.width;
    height = CamParam.CamSize.height;
    cout << width << "x" << height << endl;

    camera = new Camera(ap.deviceID, width, height, true, 10, 20);
    Image image1 = camera->captureFrame();
    cout << "opened device" << endl;

    ui = UndistortImage(CamParam);
    arProc = ArUcoProcessor(CamParam, TARGET_WIDTH, ui);

    image = Mat(height, width, CV_8UC1, image1.data);

    if (ap.saveVideo)
    {
        writer = cv::VideoWriter("out.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 24, image.size());
    }

    string userinput;
    int count = 100;
    int smallcount = 0;
    bool targetReady = false;
    int clearBuffer = 0;
    int overallCount = 24000;
    int veryoverallCout = 0;

    Mat undistorted = image.clone();
    while (1)
    {
        processImage();

        if (ap.saveVideo)
        {
            writer << image;
        }

#ifdef DISPLAY_IMAGE
        namedWindow("Display Image", WINDOW_AUTOSIZE);
        if (!ap.saveTiming)
        {
            ui.undistortAcquiredImage(image, &undistorted);
            if (arProc.foundMarkers)
            {
                Marker m = ui.undistortMarkerPoints(arProc.detectedMarker);
                //cout << m.size() << endl;
                //cout << m[0] << endl;
                //cout << m[1] << endl;
                //cout << m[2] << endl;
                //cout << m[3] << endl;

                putText(undistorted, "1", m[0], FONT_HERSHEY_COMPLEX_SMALL, 3, Scalar(255, 0, 0), 1, cv::LINE_AA);
                putText(undistorted, "2", m[1], FONT_HERSHEY_COMPLEX_SMALL, 3, Scalar(255, 255, 0), 1, cv::LINE_AA);
                putText(undistorted, "3", m[2], FONT_HERSHEY_COMPLEX_SMALL, 3, Scalar(0, 255, 0), 1, cv::LINE_AA);
                putText(undistorted, "4", m[3], FONT_HERSHEY_COMPLEX_SMALL, 3, Scalar(0, 255, 255), 1, cv::LINE_AA);

                circle(undistorted, m[0], 5, Scalar(255, 0, 0) );
                circle(undistorted, m[1], 5, Scalar(255, 255, 0) );
                circle(undistorted, m[2], 5, Scalar(0, 255, 0) );
                circle(undistorted, m[3], 5, Scalar(0, 255, 255) );
            }            

            imshow("Display Image", undistorted);
        }
        else
        {
            imshow("Display Image", image);
        }

#endif

        char c = waitKey(10);
        if (c == 27 || c == 113)
        {
            break;
        }

        if (!ap.saveData && arProc.foundMarkers && !ap.quiet)
        {
            cout << p.getInfoString();
        }

        if (ap.saveData && arProc.foundMarkers)
        {
            if (count == 100)
            {
                testFile.flush();
                userinput.clear();
                cout << "Current Location input as: depth,azimuth, cameraazimuth" << endl;
                cin >> userinput;
                count = 0;
            }

            clearBuffer++;

            if (!targetReady)
            {
                cout << p.getInfoString();
                cout << "Save current Data? (y/n) or (u) to reinput userdata" << endl;
                char progress;
                int prog = cin.get();
                //cin >> progress;
                cout << prog << endl;
                if (!cin.fail() && (prog == 'y' || prog == '\n') && prog != 'n')
                {
                    targetReady = true;
                }
                else if (prog == 'u')
                {
                    count = 100;
                }
                if (prog != '\n')
                {
                    cin.get();
                }
                smallcount = 0;
                clearBuffer = 0;
            }

            if (targetReady && smallcount < 10 && clearBuffer > 10)
            {
                smallcount++;
                count++;
                overallCount++;
                testFile << overallCount << ',' << count << ',' << userinput << ',';
                testFile << p.getInfoString();
                cout << "SAVING measurement : " << count << endl;
                imwrite("testData/raw/" + to_string(overallCount) + ".png", original);
                imwrite("testData/undistorted/" + to_string(overallCount) + ".png", image);
                savingImageTime = clock();
                if (smallcount == 10)
                {
                    targetReady = false;
                }
            }
        }

        if (ap.saveTiming && arProc.foundMarkers)
        {
            veryoverallCout++;
            if (!ap.saveData)
            {
                imwrite("testData/timing/" + to_string(veryoverallCout) + ".jpg", original);
                savingImageTime = clock();
            }
            float mdTime = markerDetectionTime - imageAcquisitionTime;
            float pcTime = posecalculationTime - markerDetectionTime;
            float diTime = drawingImageTime - posecalculationTime;
            float siTime = savingImageTime - drawingImageTime;

            stringstream timingData;
            timingData << arProc.foundMarkers << ',' << (arProc.foundMarkers && ap.saveData) << ',' << mdTime / CLOCKS_PER_SEC << ',';
            timingData << pcTime / CLOCKS_PER_SEC << ',' << diTime / CLOCKS_PER_SEC << ',' << siTime / CLOCKS_PER_SEC << ',' << p.y << endl;
            //cout << timingData.str();
            timingFile << veryoverallCout << ',' << overallCount << ',' << timingData.str();
        }
    }

    testFile.close();
    writer.release();
    return 0;
}