import cv2
import os
import numpy as np
import time
import threading
import argparse

class MultiCameraCapture():

    def __init__(self, leftID, rightID, inputFolder = None, outputFolder=None, replayms=100):
        self.framenum = 0
        self.replayms = replayms
        self.saveImages = False
        
        self.outputFolder = outputFolder
        if outputFolder:
            self.saveImages = True
            self.leftpath = outputFolder+"Left"
            self.rightpath = outputFolder+"Right"
        if outputFolder:
            if not os.path.exists(outputFolder):
                print(os.makedirs(outputFolder))
            if not os.path.exists(self.leftpath):
                os.makedirs(self.leftpath)
            if not os.path.exists(self.rightpath):
                os.makedirs(self.rightpath)

        self.shutdownFlag = False
        self.lastFrameLock = threading.Event()
        self.lastFrameLock.clear()

        self.outputFolder = None
        self.leftID = leftID
        self.rightID = rightID

        self.replay = False
        if inputFolder:
            self.replay = True
            self.inputFolder = inputFolder
            self.replayFolder()
        else:
            self.startCameras()

        
        print("MultiCameraCapture Started")
    
    def replayFolder(self):
        
        self.lastframe = [None, None]

        self.replayRunner = threading.Thread(target=self.replayRunner, args=[(self.replayms/1000)] )
        self.replayRunner.start()

    def replayRunner(self, sleeptime):
        while (True):
            time.sleep(sleeptime)
            self.framenum = self.framenum + 1

            if self.shutdownFlag:
                break
        

    def startCameras(self):
        self.leftcam = cv2.VideoCapture(self.leftID)
        self.rightcam = cv2.VideoCapture(self.rightID)
        time.sleep(0.05)

        if (self.leftcam.isOpened()== False): 
            print("Error leftCam failed to open")
            exit()

        if (self.rightcam.isOpened()== False): 
            print("Error rightcam failed to open")
            exit()

        self.lastframe = [None, None, None, None]

        self.leftTH = threading.Thread(target=self.runner, args=[self.leftID, self.leftcam])
        self.rightTH = threading.Thread(target=self.runner, args=[self.rightID, self.rightcam])

        self.leftTH.start()
        self.rightTH.start()
        time.sleep(2)


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
        
        if self.replay:
            validFrame = False
            while not validFrame:
                f1 =  cv2.imread(self.inputFolder + "Left/" + str(self.framenum) + ".jpg",1)
                f2 =  cv2.imread(self.inputFolder + "Right/" + str(self.framenum) + ".jpg",1)
                if str(f1) == "None" or str(f2) == "None":
                    print("reached the end. Relooping images")
                    self.framenum = 0
                    time.sleep(1)  
                else:
                    validFrame = True
        else:
            self.lastFrameLock.set()
            f1 = self.lastframe[self.leftID]
            f2 = self.lastframe[self.rightID]
            self.lastFrameLock.clear()
            if self.saveImages:
                cv2.imwrite(self.leftpath + "/" + str(self.framenum) + ".jpg", f1 );
                cv2.imwrite(self.rightpath + "/" + str(self.framenum) + ".jpg", f2 );
                self.framenum = self.framenum + 1
        return [f1, f2]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MultiCamera caputre device "
                                     "webcams.\n\nPress 'q' to exit.")
    parser.add_argument("--devices", type=int, nargs=2, help="Device numbers "
                        "for the cameras that should be accessed in order "
                        " (left, right).")
    parser.add_argument("--output_folder",
                        help="Folder to write output images to.")
    parser.add_argument("--input_folder",
                        help="Input Folder for images if cameras aren't available.")
    parser.add_argument("--interval", type=int, default=1000,
                        help="Interval (ms) to take pictures in.")
    args = parser.parse_args()

    d1 = 1
    d2 = 2

    if args.devices:
        d1 = args.devices[0]
        d2 = args.devices[0]

    dev = MultiCameraCapture(d1,d2, inputFolder=args.input_folder, outputFolder=args.output_folder)
    time.sleep(2)
    while True:
        frames = dev.getLastFrames()
        cv2.imshow("left",frames[0])
        cv2.imshow("right",frames[1])

        if cv2.waitKey(args.interval) & 0xFF == ord('q'):
            break

    dev.shutdown()