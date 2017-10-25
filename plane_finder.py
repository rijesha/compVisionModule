from __future__ import print_function
import os
import argparse

from contour_utils import *
from point_utils import *
import numpy as np
import cv2
import codecs, json 
from tqdm import tqdm, trange
import math
from math import *
from MultiCameraCapture import MultiCameraCapture
import time
from matplotlib import pyplot as plt
import random


from StereoVisionLib.stereo_cameras import CalibratedPair
from StereoVisionLib.calibration import StereoCalibration


text_file = open("Output.csv", "w")

csvHeader = "inputdepth, inputx, inputy, tilt, horz_rot, vert_rot, calc_tilt, depth2d, depthpers, depthman, x2d, xpers, xman, y2d, ypers, yman, range2d, rangepers, rangeman, hor2d, horpers, horman, vert2d, vertpers, vertman\n"
text_file.write(csvHeader)

    
def origianlrectangleFinder(cnts):
    new_cnts = []
    for cnt in cnts:
        eps = 0.12*cv2.arcLength(cnt,True)
        cnt = cv2.approxPolyDP(cnt, eps, True)
        if cv2.isContourConvex(cnt) and cv2.arcLength(cnt,True) > 50 and len(cnt) == 4:
            new_cnts.append(cnt)
    return new_cnts


def averageXYZcoord(coordslst):
    range = 0
    if len(coordslst) == 0:
        return
    for coord in coordslst:
        range = range + math.sqrt(coord[0]*coord[0] + coord[1]*coord[1] + coord[2]*coord[2])
    return range/len(coordslst)

def process_2d_points(pts):
    if len(pts) != 0:
        avg_width, top_width, bottom_width = get_pixel_width(pts)
        avg_height, left_height, right_height = get_pixel_height(pts)

        print((avg_width, avg_height))
        #get angles
        hor_ang, vet_ang, expected_width, expected_height = get_angles(avg_width, avg_height, 0.70)
        depths = get_depths(avg_width, avg_height)

        depth = choose_accurate_depth(depths, hor_ang, vet_ang)

        pts3d = get_3d_points_from_2d_depth(depth, pts)

        return pts3d, hor_ang, vet_ang

def process_3d_points(pts_left, pts_right, qmat):
    if len(pts_left) != 0 and len(pts_right) == len(pts_left):
        dispa = calc_disparity_from_points(ptsleft, ptsright)
        perspective_points = cv2.perspectiveTransform(np.array([dispa]), qmat)
        manual_points = calc_coordinates_from_points(ptsleft, ptsright)

        return perspective_points[0], manual_points

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find planes from two "
                                     "webcams.\n\nPress 'q' to exit.")
    parser.add_argument("--devices", type=int, nargs=2, help="Device numbers "
                        "for the cameras that should be accessed in order "
                        " (left, right).")
    parser.add_argument("--output_folder",
                        help="Folder to write output images to.")
    parser.add_argument("--input_folder",
                        help="Input Folder for images if cameras aren't available.")
    parser.add_argument("--interval", type=int, default=1000,
                        help="Interval (ms) to take pictures in.")
    args = parser.parse_args()

    d1 = 1
    d2 = 2

    if args.devices:
        d1 = args.devices[0]
        d2 = args.devices[0]

    dev = MultiCameraCapture(d1,d2, inputFolder=args.input_folder, outputFolder=args.output_folder)

    calib = StereoCalibration(input_folder="calibrations")
    qmat = calib.disp_to_depth_mat

    print(qmat)
    print(calib.cam_mats["left"])
    print(calib.dist_coefs["left"])


    while True:
        oframes = dev.getLastFrames()
        oframes = calib.rectify(oframes)
        frames = []
        frames.append(oframes[0])
        frames.append(oframes[1])

        frames[0] = cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY)
        frames[1] = cv2.cvtColor(frames[1], cv2.COLOR_BGR2GRAY)

        image1 = cv2.cvtColor(frames[0], cv2.COLOR_GRAY2BGR)
        image2 = cv2.cvtColor(frames[1], cv2.COLOR_GRAY2BGR)

        unrect = np.concatenate(frames, axis=1)

        rect = np.concatenate(frames, axis=1)

        ret, thresh1 = cv2.threshold(frames[0],150,255,0)
        ret, thresh2 = cv2.threshold(frames[1],127,255,0)
        thresh = np.concatenate((thresh1, thresh2), axis=1)

        contours_left, contours_right = havingfun(thresh1, thresh2)
        contours_left, area = get_largest_contour(contours_left)
        contours_right, area = get_largest_contour(contours_right)

        if len(contours_left) == 0:
            left_plot_contours = contours_left
        else:
            left_plot_contours = [contours_left]

        if len(contours_right) == 0:
            right_plot_contours = contours_right
        else:
            right_plot_contours = [contours_right]
        
        image1 = cv2.cvtColor(thresh1, cv2.COLOR_GRAY2BGR)
        image2 = cv2.cvtColor(thresh2, cv2.COLOR_GRAY2BGR)

        newContours1 = cv2.drawContours(image1, left_plot_contours, -1, (0,255,0), 3)
        newContours2 = cv2.drawContours(image2, right_plot_contours, -1, (0,255,0), 3)
        newconImages = np.concatenate((newContours1, newContours2), axis=1)


        notused, cnts1_orig, hierarchy1 = cv2.findContours(thresh1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        notused, cnts2_orig, hierarchy2 = cv2.findContours(thresh2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        allContours1 = cv2.drawContours(image1, cnts1_orig, -1, (0,255,0), 3)
        allContours2 = cv2.drawContours(image2, cnts2_orig, -1, (0,255,0), 3)
        allconImages = np.concatenate((allContours1, allContours2), axis=1)

 
        ptsleft = convert_contour_into_points(contours_left)
        ptsright = convert_contour_into_points(contours_right)

        experdata = []
        if ptsleft != None and ptsright != None and len(ptsleft) == 4 and len(ptsright) == 4 :
            ptsleft = order_pts(ptsleft)
            ptsright = order_pts(ptsright)

            #Get points and angles
            pts3d_from_2d, hor_angle_2d, vert_angle_2d = process_2d_points(ptsleft)

            #print((hor_angle_2d, vert_angle_2d))
            perspective_points, man_points =  process_3d_points(ptsleft, ptsright, qmat)
            
            perspective_points = perspective_points
            
            man_points = np.array(man_points)

            perspective_plane = get_plane_from_3d_pts(perspective_points)
            pers_hor_angl, pers_ver_angle = get_angles_from_plane(perspective_plane)

            manual_plane = get_plane_from_3d_pts(man_points)
            man_hor_angl, man_ver_angle = get_angles_from_plane(manual_plane)

            #print((pers_hor_angl, pers_ver_angle))
            #Get depths and distances
            depth_2d = avg_depth_from_3d(pts3d_from_2d)
            depth_pers = avg_depth_from_3d(perspective_points)
            depth_man = avg_depth_from_3d(man_points)
            print(depth_2d, depth_pers, depth_man)

            distance_2d = avg_distance_from_3d(pts3d_from_2d)
            distance_pers = avg_distance_from_3d(perspective_points)
            distance_man = avg_distance_from_3d(man_points)

            #get translations
            translations_2d = get_avg_translations_from_3d(pts3d_from_2d)
            translations_pers = get_avg_translations_from_3d(perspective_points)
            translations_man = get_avg_translations_from_3d(man_points)

            #print(translations_2d, translations_pers, translations_man)
            #get tilt
            tilt = get_2d_tilt(ptsleft)
            experdata = [tilt,  depth_2d, depth_pers, depth_man, translations_2d[0],  translations_pers[0], translations_man[0], translations_2d[1],  translations_pers[1], translations_man[1], distance_2d, distance_pers, distance_man, hor_angle_2d, pers_hor_angl, man_hor_angl, vert_angle_2d, pers_ver_angle, man_ver_angle ]
            #print(tilt)


        #cv2.imshow("Unrectified",unrect)
        #cv2.imshow("Rectified",rect)
        cv2.imshow("Threshold",thresh)
        cv2.imshow("New Contours", newconImages)
        
        
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break
        experdata = []
        if experdata != []:
            saveData = input("Save Data? (yes, YES, y, Y, Yes) \n")
            if saveData in ['yes', 'Yes', 'YES', 'y', 'Y']:
                setTilt = input("Tilt")
                setdepth = input("set_depth")
                setx = input("set_x")
                sety = input("set_y")
                horz_rot = input("horz_rot")
                vert_rot = input("vert_rot")
                userdata = [setdepth, setx, sety, setTilt, horz_rot, vert_rot]
                strexperdata = ['{:.6f}'.format(x) for x in experdata]
                csv = ",".join(userdata + strexperdata)
                print(csv)
                text_file.write(csv)
                text_file.write("\n")


    text_file.close()    
    dev.shutdown()
