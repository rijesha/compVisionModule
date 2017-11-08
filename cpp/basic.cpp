#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "target_finder.h"
#include "target_processor.h"
#include "configuration.h"

using namespace cv;

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    VideoCapture vCap(0);
    Mat image;

    Mask m1(MASK_1_LOWER_BOUND, MASK_1_UPPER_BOUND, image);
    Mask m2(MASK_2_LOWER_BOUND, MASK_2_UPPER_BOUND, image);
    Mask m3(MASK_3_LOWER_BOUND, MASK_3_UPPER_BOUND, image);

    TargetFinder tf(m1,m2,m3);

    while(1) {
        vCap.read(image);
        if ( !image.data )
        {
            printf("No image data \n");
            return -1;
        }
        tf.processNewImage(image);
        
        vector<Point> pts = tf.processNewImage(image);
        if (pts.size() > 0)
            Target t(pts);

        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", image);
        
    
        char c = waitKey(30);
        if (c == 27 || c == 113){
            break;
        }
    }
    

    return 0;
}