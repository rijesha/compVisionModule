from __future__ import print_function
import os

import numpy as np
import cv2
import codecs, json 
from tqdm import tqdm, trange

from MultiCameraCapture import MultiCameraCapture
import time
from matplotlib import pyplot as plt

if __name__ == "__main__":
    displaydisparity = True
    fig = plt.figure()

    if os.path.isfile('cameracalibrationparameters.json'):
        path = 'cameracalibrationparameters.json'
        calibratecamera = True
        obj_text = codecs.open(path, 'r', encoding='utf-8').read()
        b_new = json.loads(obj_text)
        dist_coefs_1 = np.array(b_new.get('dist_coeff_1'))
        dist_coefs_2 = np.array(b_new.get('dist_coeff_2'))
        camera_matrix_1 = np.array(b_new.get('camera_matrix_1'))
        camera_matrix_2 = np.array(b_new.get('camera_matrix_2'))


    dev = MultiCameraCapture(1,2)
    time.sleep(2)

    x = 0
    if calibratecamera:
        while x == 0 :
            frames = dev.getLastFrames()
            h,  w = frames[0].shape[:2]
            newcameramtx_1, roi_1 = cv2.getOptimalNewCameraMatrix(camera_matrix_1, dist_coefs_1, (w, h), 1, (w, h))
            newcameramtx_2, roi_2 = cv2.getOptimalNewCameraMatrix(camera_matrix_2, dist_coefs_2, (w, h), 1, (w, h))
            x, y, w, h = roi_1

    if displaydisparity:
        frames[0] = cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY)
        frames[1] = cv2.cvtColor(frames[1], cv2.COLOR_BGR2GRAY)
        stereo = cv2.StereoBM_create(0,21)
        disparity = stereo.compute(frames[1],frames[0])
        im = plt.imshow(disparity,'gray')
        firstrun = False
        plt.ion()
        plt.show()
        print("finished plotting")


    firstrun = True

    while True:
        frames = dev.getLastFrames()

        if calibratecamera:
            frames[0] = cv2.undistort(frames[0], camera_matrix_1, dist_coefs_1, None, newcameramtx_1)
            frames[0] = frames[0][y:y+h, x:x+w]
            frames[1] = cv2.undistort(frames[1], camera_matrix_2, dist_coefs_2, None, newcameramtx_2)
            frames[1] = frames[1][y:y+h, x:x+w]

            frames[0] = cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY)
            frames[1] = cv2.cvtColor(frames[1], cv2.COLOR_BGR2GRAY)
            cv2.imshow("cam1",frames[0])
            cv2.imshow("cam2",frames[1])

        if displaydisparity:
            #stereo = cv2.StereoBM_create(cv2.STEREO_BM_NARROW_PRESET,ndisparities=0, SADWindowSize=17)
            disparity = stereo.compute(frames[1],frames[0])
            im.set_data(disparity)
            plt.draw()
            plt.pause(0.01)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    dev.shutdown()