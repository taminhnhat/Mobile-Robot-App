import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
img = cv.imread("../media/raw.jpg")
print("image size: ",img.shape[1],"x",img.shape[0])
img = cv.resize(img,(200,150))
print("image size: ",img.shape[1],"x",img.shape[0])
# cv.namedWindow('src',cv.WINDOW_NORMAL)
# cv.namedWindow('gray',cv.WINDOW_NORMAL)
cv.imshow('src', img)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('gray', gray)
# Find the chess board corners
ret, corners = cv.findChessboardCorners(gray, (7,6), None)
print("chess board corners: ",ret)
# If found, add object points, image points (after refining them)
cv.waitKey(5000)
if ret == True: 
    objpoints.append(objp)
    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners2)
    # Draw and display the corners
    cv.drawChessboardCorners(img, (7,6), corners2, ret)
    cv.imshow('res', img)
cv.waitKey(0)
cv.destroyAllWindows()