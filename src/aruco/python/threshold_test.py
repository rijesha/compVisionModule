#!/usr/bin/env python
# Python 2/3 compatibility
from __future__ import print_function
import os

import numpy as np
import cv2
from time import sleep


if __name__ == '__main__':

    img = cv2.imread('3305.jpg')
    img2 = img
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("cam",img)
    cv2.imwrite("3343_bw.jpg",img)
    sleep(1)
    

    adaptiveThreshWinSizeMin = 3
    adaptiveThreshWinSizeMax = 23
    adaptiveThreshWinSizeStep = 10
    adaptiveThreshConstant = 7

    numberOfThresh = int((adaptiveThreshWinSizeMax - adaptiveThreshWinSizeMin)/adaptiveThreshWinSizeStep) +1
    thresh = []
    contoured = []
    for i in range(0,numberOfThresh):
        scale = adaptiveThreshWinSizeMin + i * adaptiveThreshWinSizeStep
        temp = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C ,cv2.THRESH_BINARY_INV, scale, adaptiveThreshConstant)
         #   _findMarkerContours(thresh, (*candidatesArrays)[i], (*contoursArrays)[i],
          #                      params->minMarkerPerimeterRate, params->maxMarkerPerimeterRate,
           ##                    params->minDistanceToBorder);
        im2, contours, hierarchy = cv2.findContours(temp, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        temp2 = img2.copy()
        cv2.drawContours(temp2, contours, -1, (0,128,0), 1)
        contoured.append(temp2)
        thresh.append(temp)
        istr = i.__str__()
        cv2.imshow(i.__str__(),temp)
        cv2.imshow(i.__str__() + "contoured",temp2)
        cv2.imwrite("3343_thresh_"+ i.__str__() +".jpg",temp)
        cv2.imwrite("3343_thresh_"+ i.__str__() +"_contoured.jpg",temp2)
    
    
    while True:
        cv2.imshow("image",img)

        if cv2.waitKey(1000) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
