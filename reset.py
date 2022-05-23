#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class Reset:

        def __init__(self):

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist.angular.z = 0.0
                self.twist.linear.x = 0.0
                self.cmd_vel_pub.publish(self.twist)

rospy.init_node('reset')
reset = Reset()
rospy.spin()
