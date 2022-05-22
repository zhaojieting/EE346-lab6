from tkinter import image_names

from matplotlib.pyplot import gray
import rospy, cv2, cv_bridge, numpy

img = cv2.imread("/home/tf/catkin_ws/EE346/src/lab5/images/cam_view1.png")
ref = cv2.imread("/home/tf/catkin_ws/EE346/src/lab5/images/bev1.jpg")
ref = cv2.resize(ref, (320,240), cv2.INTER_AREA)

pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
pts_dst = numpy.array([[30, 239],[290, 239],[30, 180],[290, 180]])
homography, status = cv2.findHomography(pts_src, pts_dst)
bev = cv2.warpPerspective(img, homography, (320,240))

gray = cv2.cvtColor(bev, cv2.COLOR_RGB2GRAY)
flag, gray_thresh = cv2.threshold(gray,110,255,cv2.THRESH_BINARY)
gray = 255-gray_thresh

hsv = cv2.cvtColor(bev, cv2.COLOR_RGB2HSV)
lower_black = numpy.array([0, 0, 0])
upper_black = numpy.array([90, 180, 150])
hsv_mask = cv2.inRange(hsv, lower_black, upper_black)

cv2.imshow('origin', img)
cv2.imshow('gray', gray)
cv2.imshow('hsv', hsv_mask)
cv2.imshow('ref', ref)
cv2.imshow('bev', bev)
cv2.waitKey()