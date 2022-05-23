import rospy, cv2, cv_bridge, numpy

img = cv2.imread("/home/tf/catkin_ws/EE346/src/lab6/images/cam_view2.png")
#ref = cv2.imread("/home/tf/catkin_ws/EE346/src/lab6/images/bev1.jpg")
#ref = cv2.resize(ref, (320,240), cv2.INTER_AREA)

#<------------------------------------------------------------------->#
#Both Lines
#pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
#pts_dst = numpy.array([[45, 239],[275, 239],[45, 220],[275,220]])

#Right Line
pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
pts_dst = numpy.array([[-10, 239],[160, 239],[-10, 220],[160,220]])

homography, status = cv2.findHomography(pts_src, pts_dst)
bev = cv2.warpPerspective(img, homography, (320,240))

#<------------------------------------------------------------------->#

gray = cv2.cvtColor(bev, cv2.COLOR_RGB2GRAY)
#Right Line
gray[gray<10] = 200

flag, gray_thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#gray_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 3)
gray_mask = 255-gray_thresh


cv2.imshow('origin', img)
cv2.imshow('gray_bev', gray)
cv2.imshow('gray', gray_mask)
#cv2.imshow('ref', ref)
cv2.imshow('bev', bev)
cv2.waitKey()