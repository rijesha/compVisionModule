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
from tqdm import tqdm, trange

from common import splitfn
import common
from MultiCameraCapture import MultiCameraCapture
import time

def getPoints(imageArray):
    bar = trange(len(imageArray))
    h, w = 0, 0
    foundArray = []
    for i in bar:
        img = imageArray[i-1]
        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            foundArray.append(img)
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            gray1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            cv2.cornerSubPix(gray1, corners, (5, 5), (-1, -1), term)

        if not found:
            tqdm.write('chessboard not found in image ' + str(i))
            continue

        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)

    return h, w, img_points, obj_points, foundArray


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

    pattern_size = (9, 6)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    img_names_undistort = []

    dev = MultiCameraCapture(1,2)
    time.sleep(2)
    cam1images = []
    cam2images = []

    i = 0
    while i < 20:
        frames = dev.getLastFrames()
        cv2.imshow("cam1",frames[0])
        cv2.imshow("cam2",frames[1])
        cam1images.append(frames[0])
        cam2images.append(frames[1])
        i = i +1
        if cv2.waitKey(1000) & 0xFF == ord('q'):
            break

    dev.shutdown()

    h, w, img_points, obj_points, foundArray = getPoints(cam1images)
    # calculate camera distortion
    rms, camera_matrix_1, dist_coefs_1, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)

  

    print("\nRMS:", rms)
    print("camera matrix:\n", camera_matrix_1)
    print("distortion coefficients: ", dist_coefs_1.ravel())

    

    h, w, img_points, obj_points, foundArray= getPoints(cam2images)
    # calculate camera distortion
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)

    print("\nRMS:", rms)
    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients: ", dist_coefs.ravel())

    data = {"camera_matrix_1": camera_matrix_1.tolist(), "dist_coeff_1": dist_coefs_1.tolist(), "camera_matrix_2": camera_matrix.tolist(), "dist_coeff_2": dist_coefs.tolist()}
    fname = "cameracalibrationparameters.json"
    import json
    with open(fname, "w") as f:
        json.dump(data, f)

    j = 0
    for img in foundArray:
        j = j + 1
        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 1, (w, h))
        dst = cv2.undistort(img, camera_matrix, dist_coefs, None, newcameramtx)

        # crop and save the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        outfile = 'output/' +  str(j) + '_undistorted.png'
        print('Undistorted image written to: %s' % outfile)
        cv2.imwrite(outfile, dst)
        cv2.imwrite('output/' + str(j) + '.png', img)

    cv2.destroyAllWindows()
