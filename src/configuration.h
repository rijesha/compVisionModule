#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp> 
using namespace cv;

#define PRINT_INFO_STRING
#define DISPLAY_IMAGE 1

#define TARGET_WIDTH 0.150
#define DEFAULT_DICTIONARY_NAME aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250
#define MARKER_ID 17

#endif /* CONFIGURATION_H */
