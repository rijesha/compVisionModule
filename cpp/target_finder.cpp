#include "target_finder.h"

RNG rng(12345);

TargetFinder::TargetFinder(){}

TargetFinder::TargetFinder(Mask m1, Mask m2, Mask m3){
    this->m1 = m1;
    this->m2 = m2;
    this->m3 = m3;
    this->imageHSL = m1.mask.clone();
}

vector<Point> TargetFinder::processNewImage(Mat orig){
    Mat img = this->imageHSL;
    cvtColor(orig, this->imageHSL, CV_BGR2HSV, 0 );
    
    inRange(img,m1.lowerbound, m1.upperbound,m1.mask);
    inRange(img,m2.lowerbound, m2.upperbound,m2.mask);
    inRange(img,m3.lowerbound, m3.upperbound,m3.mask);
    
    bitwise_or(m1.mask,m2.mask,m2.mask);
    bitwise_or(m3.mask,m2.mask,m2.mask);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(m2.mask, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    contours = findLargestRectangles(contours);
    
    for( size_t i = 0; i< contours.size(); i++ ) {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(orig, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    if (contours.size() == 0)
        return vector<Point>();
    return contours.front();
}

    
vector<vector<Point>> TargetFinder::findLargestRectangles(vector<vector<Point>> contours) {
    bool foundcontour = false;
    vector<Point> largestrectcontour;
    vector<Point> contour;
    double minArea = 50;
    
    for( size_t i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        contour = contours.at(i);
        vector<Point> approx;
        
        double eps = 0.12*arcLength(contour, true);
        approxPolyDP(contour, approx, eps, true);
        
        double currentArea = contourArea(approx);
        if (isContourConvex(approx) && arcLength(approx,true) > 25 && currentArea > minArea && approx.size() == 4){
            largestrectcontour = approx;
            minArea = currentArea;
            foundcontour = true;
        }
    }
    contours = vector<vector<Point>>();
    if (foundcontour)
        contours.push_back(largestrectcontour);
    return contours;
}

Mask::Mask(){}

Mask::Mask(Scalar lowerbound, Scalar upperbound, Mat sampleimage){
    for (int i = 0; i < 3; i ++){
        this->upperbound = upperbound;
        this->lowerbound = lowerbound;
        this->mask = sampleimage.clone();
    }
}

void Mask::printContents(void){
    cout << "lowerbound : " << this->lowerbound << " " << endl;
    cout << "upperbound : " << this->upperbound << " " << endl;
}
