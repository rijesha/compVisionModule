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

    dev = MultiCameraCapture(1,2)
    time.sleep(2)
    i = 0

    while True:
        frames = dev.getLastFrames()
        cv2.imshow("cam1",frames[0])
        cv2.imwrite('output/LEFT/' +str(i) + '.png',frames[0])
        cv2.imshow("cam2",frames[1])
        cv2.imwrite('output/RIGHT/' +str(i) + '.png',frames[1])
        i = i +1

        if cv2.waitKey(1000) & 0xFF == ord('q'):
            break

    dev.shutdown()