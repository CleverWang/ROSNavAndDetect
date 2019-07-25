import numpy as np
import cv2
import glob
import os

image_path = "/home/robot/CapturedImages/"

# remove previous detected images
remove_images = glob.glob(image_path + 'rgb*detect.jpg')
for image in remove_images:
    os.remove(image)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

images = glob.glob(image_path + 'rgb*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (6,8),None)
    
    # If found, add object points, image points (after refining them)
    if ret == True:
        ptLeftTop = (corners[0][0][0], corners[0][0][1])
        ptRightBottom = (corners[47][0][0], corners[47][0][1])
        point_color = (0, 0, 255) # BGR 
        thickness = 2 
        lineType = 8 
        cv2.rectangle(img, ptLeftTop, ptRightBottom, point_color, thickness, lineType)
        cv2.imshow('img',img)
        cv2.waitKey(1000)

        fname_n = fname[:fname.rfind('.')]
        print("found: " + fname)
        print("\twrite to " + fname_n + '_detect.jpg')
        cv2.imwrite(fname_n + '_detect.jpg', img)


        #corners2 = cv2.cornerSubPix(gray,corners,(20,20),(-1,-1),criteria)
        #print(corners2)
        # Draw and display the corners
        #img = cv2.drawChessboardCorners(img, (6,8), corners, ret)
        #cv2.imshow('img',img)
        #cv2.waitKey(1000)
    else:
        print("not found: " + fname)

cv2.destroyAllWindows()
