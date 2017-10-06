import cv2
import numpy as np
import time
import threading


class MultiCameraCapture():

    def __init__(self, cam1ID, cam2ID):
        self.shutdownFlag = False
        self.lastFrameLock = threading.Event()
        self.lastFrameLock.clear()

        self.cam1ID = cam1ID
        self.cam2ID = cam2ID

        self.cam1 = cv2.VideoCapture(cam1ID)
        self.cam2 = cv2.VideoCapture(cam2ID)
        time.sleep(0.05)

        if (self.cam1.isOpened()== False): 
            print("Error cam1 failed to open")
            exit()

        if (self.cam2.isOpened()== False): 
            print("Error cam2 failed to open")
            exit()

        self.lastframe = [None, None, None, None]

        self.cam1TH = threading.Thread(target=self.runner, args=[cam1ID, self.cam1])
        self.cam2TH = threading.Thread(target=self.runner, args=[cam2ID, self.cam2])

        self.cam1TH.start()
        self.cam2TH.start()
        print("MultiCameraCapture Started")


    def runner(self, camID, cam):
        while (True):
            ret, frame = cam.read()
            if ~self.lastFrameLock.isSet():
                self.lastframe[camID] = frame

            if self.shutdownFlag:
                break

    def shutdown(self):
        self.shutdownFlag = True

    def getLastFrames(self):
        self.lastFrameLock.set()
        f1 = self.lastframe[self.cam1ID]
        f2 = self.lastframe[self.cam2ID]
        self.lastFrameLock.clear()
        return [f1, f2]

if __name__ == "__main__":
    dev = MultiCameraCapture(1,2)
    time.sleep(2)
    while True:
        frames = dev.getLastFrames()
        cv2.imshow("cam1",frames[0])
        cv2.imshow("cam2",frames[1])

        if cv2.waitKey(1000) & 0xFF == ord('q'):
            break

    dev.shutdown()