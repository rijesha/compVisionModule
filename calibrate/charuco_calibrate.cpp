#include <opencv2/opencv.hpp>
#include "../src/configuration.h"
#include <opencv2/aruco/charuco.hpp> 

using namespace cv;
using namespace std;
Ptr<aruco::CharucoBoard> charucoboard;
Ptr<aruco::Board> board;

void makeCharucoBoard(){

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(DEFAULT_DICTIONARY_NAME);

    // create charuco board object
    charucoboard = aruco::CharucoBoard::create(22, 16, 4, 2.5, dictionary);
    board = charucoboard.staticCast<aruco::Board>();

    cv::Mat boardImage; 
    charucoboard->draw(cv::Size(2400, 1800), boardImage, 20, 1 );
    imwrite("charuco.png", boardImage);
}

int mai23n(void ){
    VideoCapture vCap = VideoCapture(DEVICE_ID);
    vCap.set(CV_CAP_PROP_FRAME_WIDTH,CAM_WIDTH);
    vCap.set(CV_CAP_PROP_FRAME_HEIGHT,CAM_HEIGHT);
    makeCharucoBoard();
    Mat image;
    while(1) {
        vCap.read(image);
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", image);

        std::vector< std::vector<cv::Point2f> > allCharucoCorners; 
        std::vector< std::vector<int> > allCharucoIds; // Detect charuco board from several viewpoints and fill allCharucoCorners and allCharucoIds ... ...
        
        // After capturing in several viewpoints, start calibration 
        cv::Mat cameraMatrix, distCoeffs;
        std::vector< Mat > rvecs, tvecs; 
        // int calibrationFlags = ... // Set calibration flags (same than in calibrateCamera() function)

        //double repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);



        char c = waitKey(2000);
        if (c == 27 || c == 113){
            break;
        }
    }
}