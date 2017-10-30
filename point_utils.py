from __future__ import print_function
import os
import cv2
import numpy as np
from math import *

def bind_number(num, minvalue, maxvalue):
    if num > maxvalue:
        return maxvalue
    if num < minvalue:
        return minvalue
    return num

def convert_contour_into_points(contour):
    if contour != []:
        pts = []
        for c in contour:
            pts.append((c[0][0],c[0][1]))
        return pts
    else :
        return []
    
def swap_smaller_point_left(pts, ind1, ind2, direction,rec = True):
    if pts[ind2][direction] < pts[ind1][direction]:
        temp = pts[ind1]
        pts[ind1] = pts[ind2]
        pts[ind2] = temp

        if ind1 != 0 and rec == True:
            pts = swap_smaller_point_left(pts, ind1-1,ind2-1, direction)
    return pts

def order_pts(pts):
    #order coordinates by x
    pts = swap_smaller_point_left(pts,0,1,0)
    pts = swap_smaller_point_left(pts,1,2,0)
    pts = swap_smaller_point_left(pts,2,3,0)
 
    #order coordinates by x and y
    pts = swap_smaller_point_left(pts,0,1,1, False)
    pts = swap_smaller_point_left(pts,3,2,1, False)

    return pts

def get_2d_tilt(pts):
    
    n = []
    n.append((pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]))
    n.append((pts[2][0] - pts[1][0], pts[2][1] - pts[1][1]))
    n.append((pts[3][0] - pts[2][0], pts[3][1] - pts[2][1]))
    n.append((pts[0][0] - pts[3][0], pts[0][1] - pts[3][1]))
    m =[]
    p = n[0]
    m.append(degrees(-atan(p[0]/p[1])))
    p = n[1]
    m.append(degrees(atan(p[1]/p[0])))
    p = n[2]
    m.append(degrees(-atan(p[0]/p[1])))
    p = n[3]
    m.append(degrees(atan(p[1]/p[0])))

    m = sum(m)/len(m)
    return m

def get_distance_between_2_points(p1, p2):
    return sqrt(pow(p1[0]-p2[0],2) + pow(p1[1]-p2[1],2))

def get_pixel_height(ordered_pts):
    left_height = get_distance_between_2_points(ordered_pts[0], ordered_pts[1])
    right_height = get_distance_between_2_points(ordered_pts[2], ordered_pts[3])
    avg_height = (left_height + right_height)/2
    return avg_height, left_height, right_height

def get_pixel_width(ordered_pts):
    top_width = get_distance_between_2_points(ordered_pts[2], ordered_pts[1])
    bottom_width = get_distance_between_2_points(ordered_pts[0], ordered_pts[3])
    avg_width = (top_width + bottom_width)/2
    return avg_width, top_width, bottom_width

def pixel_length_to_depth(pixel_length, one_meter_pixels, fitted_m=1, fitted_b=0):
    return ((one_meter_pixels / pixel_length) - fitted_b) / fitted_m

def get_horizontal_ratio(left_distance, right_distance, physical_width=0.147):
    return (left_distance - right_distance)/physical_width

def get_3d_points_from_2d_depth(depth, pts, cx=320, cy=240):
    pts_3d = []
    for p in pts:
        px = (p[0]-cx)*.002466*depth
        py = (p[1]-cy)*.002466*depth
        pts_3d.append((px,py,depth))
    return pts_3d

def avg_distance_from_3d(pts3d):
    out = 0
    for i in range(0,len(pts3d)):
        out = out + sqrt(pow(pts3d[i][0],2) + pow(pts3d[i][1],2) + pow(pts3d[i][2],2))
    return out/len(pts3d)

def get_avg_translations_from_3d(pts3d):
    x = 0
    y = 0
    for p in pts3d:
        x = x + p[0]
        y = y + p[1]
    return (x/len(pts3d), y/len(pts3d))