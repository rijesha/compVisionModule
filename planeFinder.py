from __future__ import print_function
import os

import numpy as np
import cv2
import codecs, json 
from tqdm import tqdm, trange
import math
from MultiCameraCapture import MultiCameraCapture
import time

from StereoVisionLib.stereo_cameras import CalibratedPair
from StereoVisionLib.calibration import StereoCalibration

fX = -512
fY = -512
cX = 320
cY = -240
baseLine = 60

def coordinateFinder(cornerContours):
    out = []
    for cnt in cornerContours:
        cnleft = cnt[0]
        cnright = cnt[1]

        c = []
        c.append([cnleft[0], cnright[0]])
        c.append([cnleft[1], cnright[1]])
        c.append([cnleft[2], cnright[2]]) 
        c.append([cnleft[3], cnright[3]])
        
        for i in c:
            left  = i[0]
            right = i[1]
            leftx = left[0][0]
            lefty = left[0][1]
            rightx = right[0][0]
            righty = right[0][1]

            z_out = fX * baseLine / (leftx - rightx);
            x_out = (leftx - cX) * z_out / fX;                
            y_out = (lefty - cY) * z_out / fY;  
            out.append((x_out, y_out, z_out))

    return out
        #z_out = fX * baseLine / (left.x - right.x);        
        #x_out = (left.x - cX) * z_out / fX;                
        #y_out = (left.y - cY) * z_out / fY;                

        #depth /= 1000.f; //milli-meter to meter


def rectangleFinder(cnts):
    new_cnts = []
    for cnt in cnts:
        eps = 0.12*cv2.arcLength(cnt,True)
        cnt = cv2.approxPolyDP(cnt, eps, True)
        if cv2.isContourConvex(cnt) and cv2.arcLength(cnt,True) > 50 and len(cnt) == 4:
            new_cnts.append(cnt)
    return new_cnts

def getContourXYcoordinates(cnt):
    x = []
    y = []
    for pnt in cnt:
        x.append(pnt[0][0])
        y.append(pnt[0][1])

    return x, y

def sumabslisterr(ls1, ls2):
    r = 0
    l = len(ls1)
    for i in range(0,l):
        r = r + abs(ls1[i] - ls2[i])
    return r

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

def averageXYZcoord(coordslst):
    range = 0
    if len(coordslst) == 0:
        return
    for coord in coordslst:
        range = range + math.sqrt(coord[0]*coord[0] + coord[1]*coord[1] + coord[2]*coord[2])
    return range/len(coordslst)

if __name__ == "__main__":

    dev = MultiCameraCapture(1,2)
    time.sleep(2)

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

        unrect = np.concatenate(frames, axis=1)

        rect = np.concatenate(frames, axis=1)

        ret, thresh1 = cv2.threshold(frames[0],127,255,0)
        ret, thresh2 = cv2.threshold(frames[1],127,255,0)
        thresh = np.concatenate((thresh1, thresh2), axis=1)

        image1, cnts1, hierarchy1 = cv2.findContours(thresh1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        image2, cnts2, hierarchy2 = cv2.findContours(thresh2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


        image1 = cv2.cvtColor(frames[0], cv2.COLOR_GRAY2BGR)
        image2 = cv2.cvtColor(frames[1], cv2.COLOR_GRAY2BGR)

        cnts1 = rectangleFinder(cnts1)
        cnts2 = rectangleFinder(cnts2)

        corcnts, cntleft, cntright = findCorrespondingContours(cnts1,cnts2)
        #print(coordinateFinder(corcnts))
        print(averageXYZcoord(coordinateFinder(corcnts)))
        conimg1 = cv2.drawContours(image1, cntleft, -1, (0,255,0), 3)
        conimg2 = cv2.drawContours(image2, cntright, -1, (0,255,0), 3)
        conImages = np.concatenate((conimg1, conimg2), axis=1)

        cv2.imshow("Unrectified",unrect)
        cv2.imshow("Rectified",rect)
        cv2.imshow("Threshold",thresh)
        cv2.imshow("conImages",conImages)
        
        
        
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    dev.shutdown()