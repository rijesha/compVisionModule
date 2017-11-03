import cv2
import os
import numpy as np
import time
import threading
import argparse

class CameraCapture():

    def __init__(self, camID, input_folder = None, output_folder=None, replayms=100):
        self.framenum = 0
        self.replayms = replayms
        self.saveImages = False
        
        self.output_folder = output_folder
        if output_folder:
            self.save_images = True
            self.output_path = output_folder
        if output_folder:
            if not os.path.exists(output_folder):
                print(os.makedirs(output_folder))
        
        self.shutdown_flag = False
        self.lastFrameLock = threading.Event()
        self.lastFrameLock.clear()

        self.camID = camID
        
        self.replay = False
        if input_folder:
            self.replay = True
            self.input_folder = input_folder
            self.replayFolder()
        else:
            self.startCamera()

        
        print("MultiCameraCapture Started")
    
    def replayFolder(self):
        self.lastframe = [None, None]

        self.replayRunner = threading.Thread(target=self.replayRunner, args=[(self.replayms/1000)] )
        self.replayRunner.start()

    def replayRunner(self, sleeptime):
        while (True):
            time.sleep(sleeptime)
            self.framenum = self.framenum + 1

            if self.shutdown_flag:
                break

    def startCamera(self):
        self.cam = cv2.VideoCapture(self.camID)
        time.sleep(0.05)

        if (self.cam.isOpened()== False): 
            print("Error Cam failed to open")
            exit()

        self.lastframe = [None, None, None, None]

        self.camTH = threading.Thread(target=self.runner, args=[self.camID, self.cam])

        self.camTH.start()
        time.sleep(2)


    def runner(self, camID, cam):
        while (True):
            ret, frame = cam.read()
            if ~self.lastFrameLock.isSet():
                self.lastframe[camID] = frame

            if self.shutdown_flag:
                break

    def shutdown(self):
        self.shutdown_flag = True

    def getLastFrames(self):
        if self.replay:
            validFrame = False
            while not validFrame:
                f1 =  cv2.imread(self.input_folder + str(self.framenum) + ".jpg",1)
                if str(f1) == "None" :
                    print("reached the end. Relooping images")
                    self.framenum = 0
                    time.sleep(1)  
                else:
                    validFrame = True
        else:
            self.lastFrameLock.set()
            f1 = self.lastframe[self.camID]
            self.lastFrameLock.clear()
            if self.saveImages:
                self.saveFrame()
                
        return [f1]

    def saveLastFrame(self):
        self.lastFrameLock.set()
        f1 = self.lastframe[self.camID]
        self.lastFrameLock.clear()
        
        print(self.output_folder)
        print(self.framenum)
        cv2.imwrite(self.output_folder + "/" + str(self.framenum) + ".jpg", f1 )
        self.framenum = self.framenum + 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MultiCamera caputre device "
                                     "webcams.\n\nPress 'q' to exit.")
    parser.add_argument("--devices", type=int, nargs=1, help="Device number "
                        "for the cameras that should be accessed")
    parser.add_argument("--output_folder",
                        help="Folder to write output images to.")
    parser.add_argument("--input_folder",
                        help="Input Folder for images if camera isn't available.")
    parser.add_argument("--interval", type=int, default=30,
                        help="Interval (ms) to take pictures in.")
    args = parser.parse_args()

    d1 = 1

    if args.devices:
        d1 = args.devices[0]

    dev = CameraCapture(d1, input_folder=args.input_folder, output_folder=args.output_folder)
    time.sleep(2)
    while True:
        frames = dev.getLastFrames()
        cv2.imshow("left",frames[0])

        if cv2.waitKey(args.interval) & 0xFF == ord('q'):
            break

    dev.shutdown()