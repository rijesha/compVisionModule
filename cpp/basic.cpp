#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "undistort_image.h"
#include "configuration.h"
#include <opencv2/aruco.hpp>
#include "argparse/argparse.hpp"
#include <ctime>
#include <chrono>
#include <cstring>

using namespace cv;

ofstream testFile;
string testFileFolder;

void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}

int main(int argc, const char** argv )
{   
     // make a new ArgumentParser
    ArgumentParser parser;
    parser.addArgument("-o", "--output", 1, true);
    parser.parse(argc, argv);
    testFileFolder = parser.retrieve<string>("output");
    testFile = ofstream("TestData.csv", ofstream::out);

    testFile << "depth, x, y, ";

    cout << testFileFolder << endl;
    testFile << "hello" << endl;
    testFile << CAMPARAMS_CAMERA_MATRIX  << endl;
    testFile <<  CAMPARAMS_DIST_COEFS << endl;
    testFile << "fun" << endl;
    testFile.close();
    VideoCapture vCap(1);

    CameraParameters CAMPARAMS;
    CAMPARAMS.width = CAMPARAMS_WIDTH;
    CAMPARAMS.height = CAMPARAMS_HEIGHT;
    CAMPARAMS.camera_matrix = CAMPARAMS_CAMERA_MATRIX;
    CAMPARAMS.dist_coefs = CAMPARAMS_DIST_COEFS;

    cout << CAMPARAMS_CAMERA_MATRIX <<endl;
    cout << CAMPARAMS_DIST_COEFS << endl;
    UndistortImage ui(CAMPARAMS);

    Mat image;
    Mat markerImage;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::drawMarker(dictionary, 19, 600, markerImage, 1);


    vector< int > markerIds;
    vector< vector<Point2f> > markerCorners, rejectedCandidates;
    //aruco::DetectorParameters parameters;

    imwrite( "marker19.jpg", markerImage );
    
    vCap.read(image);
    Mat dstImg = image.clone();

    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);

    while(1) {
        
        vCap.read(image);
        if ( !image.data )
        {
            printf("No image data \n");
            return -1;
        }

        #ifdef DISPLAY_IMAGE
        
        
        //aruco::detectMarkers(image, dictionary, markerCorners, markerIds, rejectedCandidates);
        aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

        if (markerIds.size() > 0){
        
            aruco::drawDetectedMarkers(image, markerCorners, markerIds);
            vector< Vec3d > rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(markerCorners, 0.159, CAMPARAMS.camera_matrix, CAMPARAMS.dist_coefs, rvecs, tvecs);
            
            Vec3d u(3);
            Mat rmat;
            Rodrigues(rvecs,rmat);
            getEulerAngles(rmat, u);

            cout << u << endl;
            // draw axis for each marker
            for(size_t i=0; i<markerIds.size(); i++){
                cv::aruco::drawAxis(image, CAMPARAMS.camera_matrix, CAMPARAMS.dist_coefs, rvecs[i], tvecs[i], 0.159);
                cout << tvecs[i] << endl;
            }
        
        }

        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", image);
        #endif
        
    
        char c = waitKey(10);
        if (c == 27 || c == 113){
            break;
        }
    }
    

    return 0;
}