#include <cmath>
#include "target_processor.h"

Target::Target(){}

bool Target::sortPointByX (Point i, Point j) { return (i.x < j.x); }
bool Target::sortPointByY (Point i, Point j) { return (i.y < j.y); }

double Target::getDistance(Point one, Point two){
    return sqrt(pow(one.x - two.x, 2) + pow(one.y - two.y, 2));
}

PixelHeight Target::getPixelHeight(vector<Point> pts){
    PixelHeight pixelheight;
    pixelheight.leftHeight = getDistance(pts.at(0), pts.at(3));
    pixelheight.rightHeight = getDistance(pts.at(1), pts.at(2));
    pixelheight.averageHeight = (pixelheight.leftHeight + pixelheight.rightHeight)/2;
    return pixelheight;
}

void Target::sortPoints(vector<Point> pts){
    using namespace std::placeholders;
    sort(pts.begin(), pts.end(), bind(&Target::sortPointByX, this, _1, _2));
    sort(pts.begin(), pts.begin()+2, bind(&Target::sortPointByY, this, _1, _2));
    sort(pts.rbegin(), pts.rend()-2, bind(&Target::sortPointByY, this, _1, _2));
}

double Target::pixelLengthToDepth(double pixelLength, double oneMeterPixels, double fitted_m, double fitted_b) {
    return ((oneMeterPixels / pixelLength) - fitted_b) / fitted_m;
}

double Target::pixelLengthToMeters(double length, double depth, double metersPerPixel){
    return length*depth*metersPerPixel;
}

vector<Point3d> Target::get3DPoints(double depth, vector<Point> pts, double metersPerPixel, int cx, int cy){
    vector<Point3d> pts3d;
    for (Point p : pts){
        double px = pixelLengthToMeters(p.x-cx, depth, metersPerPixel);
        double py = pixelLengthToMeters(p.y-cy, depth, metersPerPixel);
        pts3d.push_back(Point3d(px,py,depth));
    }
    return pts3d;
}

Translation Target::getTranslation(vector<Point3d> pts3d){
    double x = 0;
    double y = 0;
    
    for (Point3d p : pts3d){
        x = x + p.x;
        y = y + p.y;
    }
    Translation translation;
    translation.xTranslation = x/4;
    translation.yTranslation = y/4;
    return translation;
}

double Target::getTilt(vector<Point> pts){
    vector<double> tiltAngles;
    double tilt;

    tiltAngles.push_back(   -((pts[1].x - pts[0].x) / (pts[1].y - pts[0].y))*180/3.14  );
    tiltAngles.push_back(90 -((pts[2].x - pts[1].x) / (pts[2].y - pts[1].y))*180/3.14  );
    tiltAngles.push_back(   -((pts[3].x - pts[2].x) / (pts[3].y - pts[2].y))*180/3.14  );
    tiltAngles.push_back(90 -((pts[0].x - pts[3].x) / (pts[0].y - pts[3].y))*180/3.14  );
    for (double d : tiltAngles){
        tilt += d;
    }
    return tilt/tiltAngles.size();
}

Target::Target(vector<Point> pts){
    cout << "New Points" << endl;
    sortPoints(pts);
    PixelHeight ph = getPixelHeight(pts);
    cout << ph.averageHeight << " " << ph.leftHeight << " " <<ph.rightHeight << " " << endl;
    double depth = pixelLengthToDepth(ph.averageHeight, 59.5,        1.0207, -0.0172);
    double left_distance = pixelLengthToDepth(ph.leftHeight, 59.5,   1.0207, -0.0172);
    double right_distance = pixelLengthToDepth(ph.rightHeight, 59.5, 1.0207, -0.0172);

    double largest = ((left_distance > right_distance) ? 1 : -1);
    vector<Point3d> pts3d = get3DPoints(depth, pts, 0.001699422, 320, 240);
    Translation translation = getTranslation(pts3d);
    this->pts3d = pts3d;
    this->translation = translation;
}
