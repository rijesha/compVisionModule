#ifndef TARGETFINDER_H
#define TARGETFINDER_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class Mask
{
public:
    Mask();
    Mask(Scalar lowerbound, Scalar upperbound, Mat sampleimage);
    void printContents();
    Scalar lowerbound;
    Scalar upperbound;
    Mat mask;
};


class TargetFinder
{
private:
    vector<vector<Point>> findLargestRectangles(vector<vector<Point>> contours);

public:
    TargetFinder();
    TargetFinder(Mask m1, Mask m2, Mask m3);
    vector<Point> processNewImage(Mat img);
    Mask m1;
    Mask m2;
    Mask m3;
    Mat imageHSL;
};

#endif /* TARGETFINDER_H */