#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <common/undistort_image.h>
#include "common/configuration.h"
#include <common/cvm_argument_parser.hpp>
#include <ctime>
#include <chrono>
#include <cstring>
#include <fstream>
#include <time.h>
#include <common/aruco_processor.h>
#include <unistd.h>
#include "boost/filesystem.hpp"
#include "csv.h"
using namespace boost::filesystem;

using namespace cv;

//boost::filesystem::ofstream testFile;
//string testFileFolder;

clock_t imageAcquisitionTime, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime;

VideoCapture vCap;
UndistortImage ui;
Mat image;
Mat original;

ArUcoProcessor arProc;
int multipleMarkerCount = 0;
string current_file;
Position p;

void processImage(Mat original){
    arProc.foundMarkers = false;
    
    imageAcquisitionTime = clock();
    if ( !original.data )
    {
        printf("No image data \n");
    }

    arProc.processFrame(original);
    markerDetectionTime = clock();
    p = arProc.calculatePose();
    posecalculationTime = clock();
    cvtColor(original, image, COLOR_GRAY2BGR );
    image = arProc.drawMarkersAndAxis(image);
    drawingImageTime = clock();
}

int width;
int height;

int main(int argc, const char** argv )
{   
    CVMArgumentParser ap(argc, argv, true, false, true, false);
    
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile(ap.calib_file_path);

    width = CamParam.CamSize.width;
    height = CamParam.CamSize.height;
    cout << width << "x" << height << endl;

    boost::filesystem::ofstream timingFile("timeDataFolder.csv", std::ios_base::out);
    timingFile << "veryoverallCout, overallCount, foundMarker, savedImage, markerDetectionTime, posecalculationTime, drawingImageTime, savingImageTime, depth" << endl;

    ui = UndistortImage(CamParam);
    arProc = ArUcoProcessor(CamParam, TARGET_WIDTH, ui);

    io::CSVReader<14> in(ap.inputpath);
    in.read_header(io::ignore_extra_column, "imgnum", "count", "depth", "azimuth", "camera azimuth", "tvec1", "tvec2", "tvec3", "rvec1", "rvec2", "rvec3", "wpos1", "wpos2", "wpos3");
    double imgnum, count, depth, azimuth, camera_azimuth, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3;
    while(in.read_row(imgnum, count, depth, azimuth, camera_azimuth, tvec1, tvec2, tvec3, rvec1, rvec2, rvec3, wpos1, wpos2, wpos3)){
        cout << imgnum << endl;
        current_file = "testData/raw/" + to_string((int)imgnum) + ".png";
        Mat fun = imread(current_file, IMREAD_GRAYSCALE);
        cout << current_file << endl;
        if(! fun.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;
        }
        processImage(fun);

        float mdTime = markerDetectionTime - imageAcquisitionTime;
        float pcTime = posecalculationTime - markerDetectionTime ;
        float diTime = drawingImageTime - posecalculationTime;
        float siTime = savingImageTime - drawingImageTime;
        
        stringstream timingData;
        timingData << arProc.foundMarkers << ','  << (arProc.foundMarkers && ap.saveData)  << ',' << mdTime /CLOCKS_PER_SEC  << ',' ;
        timingData << pcTime /CLOCKS_PER_SEC << ',' <<  diTime/CLOCKS_PER_SEC << ',' << siTime/CLOCKS_PER_SEC << ',' << p.y << ',' << p.azi << endl;
        //cout << timingData.str();
        timingFile << imgnum << ',' << timingData.str();

    }
    timingFile.close();


    return 0;
}
