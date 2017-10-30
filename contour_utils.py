from __future__ import print_function
import os
import cv2
import numpy as np
from math import *

def get_largest_contour(contours):
    if len(contours) == 0:
        return [], 0
    i = 0
    maxArea = 0
    index = 0
    for contour in contours:
        if cv2.contourArea(contour) > maxArea:
            maxArea = cv2.contourArea(contour)
            index = i
        i = i + 1

    return contours[index], maxArea

def findRectangles(cnts):
    new_cnts = []
    i = 0
    for cnt in cnts:
        eps = 0.12*cv2.arcLength(cnt,True)
        cnt = cv2.approxPolyDP(cnt, eps, True)
        if cv2.isContourConvex(cnt) and cv2.arcLength(cnt,True) > 25 and cv2.contourArea(cnt) > 50 and cv2.contourArea(cnt) < 300000 and len(cnt) == 4:
            new_cnts.append(cnt)
            #print(i)
            i = i+1
            #print(cv2.arcLength(cnt,True))
            #print(cv2.contourArea(cnt))

    return new_cnts

def getContourXYcoordinates(cnt):
    x = []
    y = []
    for pnt in cnt:
        x.append(pnt[0][0])
        y.append(pnt[0][1])

    return x, y
