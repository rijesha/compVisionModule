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

def get_angles(avg_width, avg_height, height_to_width_ratio):
    expected_height = avg_width*height_to_width_ratio
    expected_width = avg_height/height_to_width_ratio

    hor_ratio = avg_width/expected_width
    vert_ratio = avg_height/expected_height

    #print("width")
    #print((avg_width, expected_width))
    #print(hor_ratio)
    #print("height")
    #print((avg_height, expected_height))
    #print(vert_ratio)

    horizontal_angle = acos(bind_number(hor_ratio,-1,1))
    veritcal_angle = acos(bind_number(vert_ratio,-1,1))

    return degrees(horizontal_angle), degrees(veritcal_angle), expected_width, expected_height

def get_depths(avg_width, avg_height):
    data = (120.0/avg_width, 85.0/avg_height)
    return data

def choose_accurate_depth(depths, hor_angle, vert_angle):
    if abs(hor_angle) > abs(vert_angle):
        return depths[1]
    else:
        return depths[0]

def get_3d_points_from_2d_depth(depth, pts, cx = 320, cy = 240):
    pts_3d = []
    for p in pts:
        px = (p[0]-cx)*.002466*depth
        py = (p[1]-cy)*.002466*depth
        pts_3d.append((px,py,depth))
    return pts_3d


def calc_coordinates_from_points(leftpts, rightpts):
    fx = 677.85791904
    fy = 677.85791904
    cx = 356.74900547
    cy = 231.07630461
    b = .060

    out = []
    for i in range(0, len(leftpts)):
        delta = leftpts[i][0] - rightpts[i][0]
        z = fx*b/delta
        x = (leftpts[i][0] - cx)*z/fx
        y = (leftpts[i][1] - cy)*z/fy
        out.append((x,y,z))
    return out

def calc_disparity_from_points(leftpts, rightpts):
    out = []
    for i in range(0, len(leftpts)):
        delta = leftpts[i][0] - rightpts[i][0]
        out.append([leftpts[i][0],leftpts[i][1],delta])

    return np.array(out, dtype=np.float32)

def avg_depth_from_3d(pts3d):
    out = 0
    for i in range(0,len(pts3d)):
        out = out + pts3d[i][2]
    return out/len(pts3d)

def avg_distance_from_3d(pts3d):
    out = 0
    for i in range(0,len(pts3d)):
        out = out + sqrt(pow(pts3d[i][0],2) + pow(pts3d[i][1],2) + pow(pts3d[i][2],2))
    return out/len(pts3d)

def get_plane_from_3d_pts(pts3d):
    #https://stackoverflow.com/questions/12299540/plane-fitting-to-4-or-more-xyz-points
    xs = []
    ys = []
    zs = []
    i = 0
    for e in pts3d:
        xs.append(e[0])
        ys.append(e[1])
        zs.append(e[2])
        i = i + 1
    
    xs = np.array(xs, dtype='float')
    ys = np.array(ys, dtype='float')
    zs = np.array(zs, dtype='float')

    tmp_A = []
    tmp_b = []

    for i in range(len(xs)):
        tmp_A.append([xs[i], ys[i], 1])
        tmp_b.append(zs[i])

    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)
    
    fit = (A.T * A).I * A.T * b
    #print("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    return fit

def get_angles_from_plane(plane):
    return degrees(atan(plane[0])), degrees(atan(plane[1]))

def get_avg_translations_from_3d(pts3d):
    x = 0
    y = 0
    for p in pts3d:
        x = x + p[0]
        y = y + p[1]
    return (x/len(pts3d), y/len(pts3d))