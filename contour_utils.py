from __future__ import print_function
import os
import cv2
import numpy as np
from math import *

def sumabslisterr(ls1, ls2):
    r = 0
    l = len(ls1)
    for i in range(0,l):
        r = r + abs(ls1[i] - ls2[i])
    return r

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

def checkIfContoursAreClose(cnt1, cnt2):
    cnt1x, cnt1y = getContourXYcoordinates(cnt1)
    cnt2x, cnt2y = getContourXYcoordinates(cnt2)
    abserr = sumabslisterr(cnt1x, cnt2x) + sumabslisterr(cnt1y, cnt2y)
    
    if abserr < 200:
        return True

def findCorrespondingContours(cnts1, cnts2):
    cntleft = []
    cntright = []
    cntPairs = []
    for cnt1 in cnts1:
        for cnt2 in cnts2:
            if ~(abs(cv2.arcLength(cnt1,True) - cv2.arcLength(cnt2,True)) > 10) and checkIfContoursAreClose(cnt1, cnt2):
                cntPairs.append((cnt1,cnt2))
                cntleft.append(cnt1)
                cntright.append(cnt2)
    return cntPairs, cntleft, cntright

def havingfun(threshleft, threshright):
    notused, left_contours, left_hierarchy = cv2.findContours(threshleft,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    notused, right_contours, right_hierarchy = cv2.findContours(threshright,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    #print("left contours")
    #left_contours = findRectangles(left_contours)
    #print("right contours")
    #right_contours = findRectangles(right_contours)

    #cntPairs, left_contours, right_contours = findCorrespondingContours(left_contours, right_contours)

    return left_contours, right_contours


