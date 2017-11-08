#ifndef TARGETPROCESSOR_H
#define TARGETPROCESSOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

struct PixelHeight {
    double averageHeight;
    double leftHeight;
    double rightHeight;
};

struct Translation {
    double xTranslation;
    double yTranslation;
};

class Target
{
private:
    void sortPoints(vector<Point> pts);
    bool sortPointByX(Point i, Point j);
    bool sortPointByY(Point i, Point j);
    double getDistance(Point one, Point two);
    PixelHeight getPixelHeight(vector<Point> pts);
    double pixelLengthToDepth(double pixelLength, double oneMeterPixels, double fitted_m, double fitted_b);
    vector<Point3d> get3DPoints(double depth, vector<Point> pts, double metersPerPixel, int cx, int cy);
    double pixelLengthToMeters(double length, double depth, double metersPerPixel);
    Translation getTranslation(vector<Point3d> pts3d);
    double getTilt(vector<Point> pts);
public:
    Target();
    Target(vector<Point> pts);
    vector<Point> processNewImage(Mat img);
    vector<Point> pts;
    vector<Point3d> pts3d;
    Translation translation;
};

#endif /* TARGETPROCESSOR_H */