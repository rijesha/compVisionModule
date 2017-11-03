from __future__ import print_function
import os
import argparse

from contour_utils import *
from point_utils import *
import numpy as np
import cv2
from math import *
from camera_capture import CameraCapture
from calibrate import *

import time
import random


frame = None
text_file = None
contour_buffer = []

def get_color_of_frame(event, x, y, flags, param):
    print(frame[y][x])

def get_color_of_frame3(event, x, y, flags, param):
    if x > 1280:
        x = x -1280
    elif x > 640:
        x = x -640
    print(frame[y][x])

def nothing(x):
    pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find planes from two "
                                     "webcams.\n\nPress 'q' to exit.")
    parser.add_argument("--devices", type=int, nargs=1, help="Device numbers "
                        "for the cameras that should be accessed in order "
                        " (left, right).")
    parser.add_argument("--acquire_user_data", action='store_true', help="prompts user for ground truth "
                        "references and saves data to timestamped csv ")
    parser.add_argument("--output_folder",
                        help="Folder to write output images to.")
    parser.add_argument("--input_folder",
                        help="Input Folder for images if cameras aren't available.")
    parser.add_argument("--interval", type=int, default=1000,
                        help="Interval (ms) to take pictures in.")
    args = parser.parse_args()

    d1 = 1

    if args.devices:
        d1 = args.devices[0]

    
    dev = CameraCapture(d1, output_folder="output")

    calib_data = Calibration_Data().load_data('calibration.p')
    print(calib_data)
    undistorter = Undistort_Image(calib_data)
    
    cv2.namedWindow("control")
    cv2.namedWindow("Hue Ranges")
    cv2.createTrackbar('LowLowerH','Hue Ranges',1,5,nothing)
    cv2.createTrackbar('LowUpperH','Hue Ranges',6,10,nothing)
    cv2.createTrackbar('MidLowerH','Hue Ranges',5,10,nothing)
    cv2.createTrackbar('MidUpperH','Hue Ranges',10,20,nothing)
    cv2.createTrackbar('HighLowerH','Hue Ranges',10,20,nothing)
    cv2.createTrackbar('HighUpperH','Hue Ranges',20,30,nothing)
    

    cv2.createTrackbar('LowLowerS','control',255,255,nothing)
    cv2.createTrackbar('LowUpperS','control',255,255,nothing)
    cv2.createTrackbar('LowLowerV','control',40,255,nothing)
    cv2.createTrackbar('LowUpperV','control',255,255,nothing)

    cv2.createTrackbar('MidLowerS','control',105,255,nothing)
    cv2.createTrackbar('MidUpperS','control',255,255,nothing)
    cv2.createTrackbar('MidLowerV','control',130,255,nothing)
    cv2.createTrackbar('MidUpperV','control',255,255,nothing)

    cv2.createTrackbar('HighLowerS','control',60,255,nothing)
    cv2.createTrackbar('HighUpperS','control',255,255,nothing)
    cv2.createTrackbar('HighLowerV','control',191,255,nothing)
    cv2.createTrackbar('HighUpperV','control',255,255,nothing)

    cv2.namedWindow("cam")
    cv2.setMouseCallback("cam", get_color_of_frame)
    cv2.namedWindow("mask")
    cv2.setMouseCallback("mask", get_color_of_frame3)
    cv2.namedWindow("maskmult")
    cv2.setMouseCallback("maskmult", get_color_of_frame)

    while True:
        frame = dev.getLastFrames()[0]
        frame = undistorter.undistort(frame)
        original = frame
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        LowLowerH = cv2.getTrackbarPos('LowLowerH','Hue Ranges')
        LowUpperH = cv2.getTrackbarPos('LowUpperH','Hue Ranges')
        MidLowerH = cv2.getTrackbarPos('MidLowerH','Hue Ranges')
        MidUpperH = cv2.getTrackbarPos('MidUpperH','Hue Ranges')
        HighLowerH = cv2.getTrackbarPos('HighLowerH','Hue Ranges')
        HighUpperH = cv2.getTrackbarPos('HighUpperH','Hue Ranges')


        LowLowerV = cv2.getTrackbarPos('LowLowerV','control')
        LowLowerS = cv2.getTrackbarPos('LowLowerS','control')
        LowUpperV = cv2.getTrackbarPos('LowUpperV','control')
        LowUpperS = cv2.getTrackbarPos('LowUpperS','control')

        MidLowerV = cv2.getTrackbarPos('MidLowerV','control')
        MidLowerS = cv2.getTrackbarPos('MidLowerS','control')
        
        MidUpperV = cv2.getTrackbarPos('MidUpperV','control')
        MidUpperS = cv2.getTrackbarPos('MidUpperS','control')

        HighLowerV = cv2.getTrackbarPos('HighLowerV','control')
        HighLowerS = cv2.getTrackbarPos('HighLowerS','control')
        
        HighUpperV = cv2.getTrackbarPos('HighUpperV','control')
        HighUpperS = cv2.getTrackbarPos('HighUpperS','control')

        lowRangeLower = np.array([LowLowerH,LowLowerS,LowLowerV], dtype = "uint8")
        lowRangeUpper = np.array([LowUpperH,LowUpperS,LowUpperV], dtype = "uint8")
        lowmask = cv2.inRange(frame, lowRangeLower, lowRangeUpper)

        midRangeLower = np.array([MidLowerH,MidLowerS,MidLowerV], dtype = "uint8")
        midRangeUpper = np.array([MidUpperH,MidUpperS,MidUpperV], dtype = "uint8")
        midmask = cv2.inRange(frame, midRangeLower, midRangeUpper)

        highRangeLower = np.array([HighLowerH,HighLowerS,HighLowerV], dtype = "uint8")
        highRangeUpper = np.array([HighUpperH,HighUpperS,HighUpperV], dtype = "uint8")
        highmask = cv2.inRange(frame, highRangeLower, highRangeUpper)

        mask = np.concatenate((lowmask, midmask, highmask), axis=1)        
        cv2.imshow("mask",mask)

        maskmult = cv2.bitwise_or(cv2.bitwise_or(lowmask, midmask), highmask)
        notused, contours, left_hierarchy = cv2.findContours(maskmult,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        
        maskmult = cv2.cvtColor(maskmult, cv2.COLOR_GRAY2BGR)
        maskmult = cv2.drawContours(maskmult, contours, -1, (0,255,0), 3)
        cv2.imshow("maskmult",maskmult)
        
        contours = findRectangles(contours)
        contour, area = get_largest_contour(contours)
        if len(contour) == 0:
            contour = contour
            pts = None
            contourframe = cv2.drawContours(original, contour, -1, (0,255,0), 3)
        else:
            pts = convert_contour_into_points(contour)
            pts = order_pts(pts)
            
            contour[0] = np.resize(np.array(list(pts[0])),(1,2))
            contour[1] = np.resize(np.array(list(pts[1])),(1,2))
            contour[2] = np.resize(np.array(list(pts[2])),(1,2))
            contour[3] = np.resize(np.array(list(pts[3])),(1,2))

            contour_buffer.append(contour)
            if len(contour_buffer) == 2:
                contour = averageContourList(contour_buffer)
                contour_buffer.pop(0)
            pts = convert_contour_into_points(contour)
            pts = order_pts(pts)
            
            contour = [contour]
            contourframe = cv2.drawContours(original, contour, -1, (0,255,0), 3)
            contourframe = cv2.circle(contourframe,pts[0],5, (0,0,255), 3)
            contourframe = cv2.circle(contourframe,pts[1],5, (0,255,0), 3)
            contourframe = cv2.circle(contourframe,pts[2],5, (255,0,0), 3)
            

        

        experdata = []
        if pts != None and len(pts) == 4 :
            pts = order_pts(pts)
            
            #Get points and angles
            avg_width, top_width, bottom_width = get_pixel_width(pts)
            avg_height, left_height, right_height = get_pixel_height(pts)

            depth_from_width = pixel_length_to_depth(avg_width, 86.5, fitted_m=1.0207, fitted_b=-0.0172)
            depth_from_height = pixel_length_to_depth(avg_height, 59.5, fitted_m=1.0207, fitted_b=-0.0172)
            
            left_distance = pixel_length_to_depth(left_height, 59.5, fitted_m=1.0207, fitted_b=-0.0172)
            right_distance = pixel_length_to_depth(right_height, 59.5, fitted_m=1.0207, fitted_b=-0.0172)

            horizontal_ratio = get_horizontal_ratio(left_distance, right_distance)
            
            depth = depth_from_height
            #print(depth)
            pts3d = get_3d_points_from_2d_depth(depth, pts, cx=calib_data.camera_matrix[0][2],cy=calib_data.camera_matrix[1][2])
            #pixel_length_to_meters(avg_height, depth, meters_per_pixel=0.0017478992)
            
            distance = avg_distance_from_3d(pts3d)
            #get translations
            translations = get_avg_translations_from_3d(pts3d)
            
            #get tilt
            tilt = get_2d_tilt(pts)
            experdata = [tilt,  depth, translations[0], translations[1], distance, horizontal_ratio]
            #print(tilt)
            

        cv2.imshow("cam",contourframe)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
        pass

        if experdata != [] and args.acquire_user_data:
            saveData = input("Save Data? (yes, YES, y, Y, Yes) \n")
            if saveData in ['yes', 'Yes', 'YES', 'y', 'Y']:
                if text_file is None:
                    timestr = time.strftime("%Y%m%d-%H%M%S") + ".csv"
                    text_file = open(timestr, "w")

                    csvHeader = "setdepth, setx, sety, setTilt, sethorz_rot, tilt,  depth, translations[0], translations[1], distance, horizontal_ratio, pts\n"
                    text_file.write(csvHeader)

                dev.saveLastFrame()
                setTilt = input("Tilt?")
                setdepth = input("depth of target?")
                setx = input("translations - x?")
                sety = input("translations - y?")
                horz_rot = input("horizontal_angle?")
                userdata = [setdepth, setx, sety, setTilt, horz_rot]
                strexperdata = ['{:.6f}'.format(x) for x in experdata]
                csv = ",".join(userdata + strexperdata + [str(pts)])
                print(csv)
                text_file.write(csv)
                text_file.write("\n")


    dev.shutdown()
    if text_file is not None:
        text_file.close()
