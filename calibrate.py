#!/usr/bin/env python

'''
camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images

usage:
    calibrate.py [--debug <output path>] [--square_size] [<image mask>]

default values:
    --debug:    ./output/
    --square_size: 1.0
    <image mask> defaults to ../data/left*.jpg
'''

# Python 2/3 compatibility
from __future__ import print_function
import os

import numpy as np
import cv2

from camera_capture import CameraCapture
import time
import pickle

def getCurrentMillis():
    return int(round(time.time() * 1000))
    

class Find_Calibration_Parameters():
    def __init__(self, pattern_size = (9, 6),square_size = 1.0, obj_points = [], img_points = []) :
        self.square_size = square_size
        self.pattern_size = pattern_size
        self.pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
        self.pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
        self.pattern_points *= square_size

        self.obj_points = obj_points
        self.img_points = img_points
        self.h, self.w = 0, 0

        self.found_images = []

    def reset_points(self):
        self.obj_points = []
        self.img_points = []

    def process_new_image(self, img):
        if img is None:
            print("Image Failed")
            return None

        orig = img
        img_bw = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        self.h, self.w = img_bw.shape[:2]
        found, corners = cv2.findChessboardCorners(img_bw, self.pattern_size)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img_bw, corners, (5, 5), (-1, -1), term)
            cv2.drawChessboardCorners(img, self.pattern_size, corners, found)
            self.found_images.append(orig)
            self.img_points.append(corners.reshape(-1, 2))
            self.obj_points.append(self.pattern_points)
            return img
        else:
            print("Failed to find Chessboard")
            return None
    
    def calibrate(self, print_raw_results = True):
        rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, (self.w, self.h), None, None)
        calib_data = Calibration_Data(rms, camera_matrix, dist_coefs, self.w, self.h, rvecs, tvecs)
        if print_raw_results:
            print(calib_data)

        return calib_data

class Calibration_Data():
    def __init__(self, rms = None, camera_matrix= None, dist_coefs= None, w= None, h= None, rvecs= None, tvecs= None) :
        self.rms = rms
        self.camera_matrix = camera_matrix
        self.dist_coefs = dist_coefs
        self.w = w
        self.h = h
        self.rvecs = rvecs
        self.tvecs = tvecs
        if camera_matrix is not None:
            self.cx = camera_matrix[0][2]
            self.cy = camera_matrix[1][2]

    def load_data(self, filepath):
        return pickle.load( open( filepath, "rb" ) )

    def save_data(self, filepath):
        pickle.dump(self, open( filepath, "wb" ) )

    def __str__(self):
        s = []
        s.append("RMS")
        s.append(str(self.rms))
        s.append("camera Matrix")
        s.append(str(self.camera_matrix))
        s.append("dist_coefs")
        s.append(str(self.dist_coefs.ravel()))
        return "\n".join(s)


class Undistort_Image():
    def __init__(self, calib_data) :
        self.h = calib_data.h
        self.w = calib_data.w
        self.rms = calib_data.rms
        self.camera_matrix = calib_data.camera_matrix
        self.dist_coefs = calib_data.dist_coefs

        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coefs, (self.w, self.h), 1, (self.w, self.h))
    
    def undistort(self, img):
        return cv2.undistort(img, self.camera_matrix, self.dist_coefs, None, self.newcameramtx)

    def crop_and_undistort(self, img):
        x, y, w, h = self.roi
        return self.undistort(img)[y:y+h, x:x+w]


if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob

    args, img_mask = getopt.getopt(sys.argv[1:], '', ['debug=', 'square_size='])
    args = dict(args)
    args.setdefault('--debug', './output/')
    args.setdefault('--square_size', 1.0)
    

    debug_dir = args.get('--debug')
    if not os.path.isdir(debug_dir):
        os.mkdir(debug_dir)
    square_size = float(args.get('--square_size'))


    dev = CameraCapture(1)
    time.sleep(2)

    camimages = []
    keep_looping = True
    calib_params = Find_Calibration_Parameters()
    lastupdatetime = getCurrentMillis()
    i = 0

    while keep_looping:
        img = dev.getLastFrames()[0]
        cv2.imshow("cam",img)

        if (getCurrentMillis() - lastupdatetime) > 1000:
            ret = calib_params.process_new_image(img)
            if ret is not None:
                cv2.imshow("cam",ret)
                calib_data = calib_params.calibrate(print_raw_results = False)
                print(calib_data.rms)
                i = i + 1
                if calib_data.rms < 4 and i > 20:
                    keep_looping = False
            
            lastupdatetime = getCurrentMillis()
        
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break


    print(calib_data)
    undistorter = Undistort_Image(calib_data)

    while True:
        img = dev.getLastFrames()[0]
        cv2.imshow("cam",img)
        cv2.imshow("undistorted",undistorter.crop_and_undistort(img))

        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    calib_data.save_data('calibration.p')
    dev.shutdown()
    cv2.destroyAllWindows()
