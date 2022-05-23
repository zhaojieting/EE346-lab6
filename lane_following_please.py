#!/usr/bin/env python

import this
import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('raspicam_node/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                self.onlyRight = False

                if self.onlyRight:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[-10, 239],[160, 239],[-10, 220],[160,220]])

                else:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[45, 239],[275, 239],[45, 220],[275,220]])
                
                self.homography, self.status = cv2.findHomography(pts_src, pts_dst)

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                h, w, d = image.shape
                bev = cv2.warpPerspective(image, self.homography, (w,h))

                gray = cv2.cvtColor(bev, cv2.COLOR_RGB2GRAY)
                gray[gray<10] = 200     #把因为homography而黑色的区域变成和地面颜色相近
                flag, gray_thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                gray_mask = 255-gray_thresh

                search_top = 140
                search_bot = 200
                search_left = 20
                search_right = 260
                #gray_mask[0:search_top, 0:w] = 0
                gray_mask[0:h, 0:search_left] = 0
                #gray_mask[0:h, search_right:w] = 0
                #gray_mask[search_bot:h, 0:w] = 0
                
                curv = self.cal_curvature(gray)
                curv = max(0, numpy.mean(curv) - 0.15075)

                M = cv2.moments(gray_mask)

                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    fpt_x = cx  #(cx1 + cx2)/2
                    fpt_y = cy  #(cy1 + cy2)/2

                    cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    theta = math.atan2(err, fpt_y)

                    self.twist.angular.z =   5 * theta + 0.5 * (err*90.0/160)/15
                    if err != 0:
                            self.twist.linear.x = 0.2 - 0.5*curv
                    else:
                            self.twist.linear.x = 0.2
                    #print(4*theta,  (err*90.0/160)/15, self.twist.linear.x)
                    self.cmd_vel_pub.publish(self.twist)

                else:
                        self.twist.angular.z = 0.1
                        self.twist.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.twist)

                cv2.imshow("camera view", image)
                cv2.imshow("bird eye view",bev)
                cv2.waitKey(1)

        def cal_curvature(self, img):
                
                x , y = numpy.gradient(img)
                xx, xy = numpy.gradient(x)
                yx, yy = numpy.gradient(y)
                Iup =  (1+x*x)*yy - 2*x*y*xy + (1+y*y)*xx
                Idown = 2*numpy.power((1 + x*x + y*y),1.5)
                curv  = Iup/Idown
                curv = abs(curv)
                return curv



rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
