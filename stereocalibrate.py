import numpy as np
import cv2
import glob
from MultiCameraCapture import MultiCameraCapture
import time

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)


# Arrays to store object points and image points from all the images.
objpointsL = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
objpointsR = []
imgpointsR = []


dev = MultiCameraCapture(1,2)
time.sleep(2)
cam1images = []
cam2images = []
i = 0
while i < 10:
    frames = dev.getLastFrames()
    cv2.imshow("cam1",frames[0])
    cv2.imshow("cam2",frames[1])
    cam1images.append(frames[0])
    cam2images.append(frames[1])
    i = i +1
    if cv2.waitKey(1000) & 0xFF == ord('q'):
        break
dev.shutdown()


lFrame = cam1images[0]
rFrame = cam2images[0]
for img in cam1images:
    grayL = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, cornersL = cv2.findChessboardCorners(grayL, (9,6),None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpointsL.append(objp)

        cv2.cornerSubPix(grayL,cornersL,(11,11),(-1,-1),criteria)
        imgpointsL.append(cornersL)



for img in cam2images:
    grayR = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, cornersR = cv2.findChessboardCorners(grayR, (9,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpointsR.append(objp)

        cv2.cornerSubPix(grayR,cornersR,(11,11),(-1,-1),criteria)
        imgpointsR.append(cornersR)

mat1 = np.zeros(shape=(3,3))
mat2 = np.zeros(shape=(3,3))
dis1 = np.zeros(shape=(1,5))
dis2 = np.zeros(shape=(1,5))

returnlist = cv2.stereoCalibrate(objpointsL, imgpointsL, imgpointsR, mat1, dis1, mat2, dis2, (1280,960))

print(returnlist)

retval,cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = returnlist

# Assuming you have left01.jpg and right01.jpg that you want to rectify
lFrame = cv2.imread('left01.jpg')
rFrame = cv2.imread('right01.jpg')
w, h = lFrame.shape[:2] # both frames should be of same shape
frames = [lFrame, rFrame]

# Params from camera calibration
camMats = [cameraMatrix1, cameraMatrix2]
distCoeffs = [distCoeffs1, distCoeffs2]

camSources = [0,1]
for src in camSources:
    distCoeffs[src][0][4] = 0.0 # use only the first 2 values in distCoeffs

# The rectification process
newCams = [0,0]
roi = [0,0]
for src in camSources:
    newCams[src], roi[src] = cv2.getOptimalNewCameraMatrix(cameraMatrix = camMats[src], 
                                                           distCoeffs = distCoeffs[src], 
                                                           imageSize = (w,h), 
                                                           alpha = 0)



rectFrames = [0,0]
for src in camSources:
        rectFrames[src] = cv2.undistort(frames[src], 
                                        camMats[src], 
                                        distCoeffs[src])

# See the results
view = np.hstack([frames[0], frames[1]])    
rectView = np.hstack([rectFrames[0], rectFrames[1]])

cv2.imshow('view', view)
cv2.imshow('rectView', rectView)

# Wait indefinitely for any keypress
cv2.waitKey(0)