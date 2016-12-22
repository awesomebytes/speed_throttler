#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

"""
Given an input Twist topic
limit its speed by using a LaserScan
topic.


"""


class Throttler(object):
    def __init__(self):
        rospy.loginfo("Initializing throttler...")
        self.scan_threshold = 0.5
        self.twist_reducer = 0.5
        self.last_scan = None
        self.scan_sub = rospy.Subscriber('/scan',
                                         LaserScan,
                                         self.scan_cb,
                                         queue_size=1)

        self.reduced_twist_pub = rospy.Publisher('/reduced_twist',
                                                 Twist,
                                                 queue_size=1)

        self.twist_sub = rospy.Subscriber('/base_controller/command',
                                          Twist,
                                          self.twist_cb,
                                          queue_size=1)

    def scan_cb(self, s):
        """
        :param LaserScan s: laser scan
        """
        self.last_scan = s

    def twist_cb(self, t):
        """
        :param Twist t: twist command
        """
        # Check on the last scan message
        # if any point is under self.scan_threshold
        # if so, apply self.twist_reducer
        # and publish
        

        self.reduced_twist_pub.publish(t)


if __name__ == '__main__':
    rospy.init_node('speed_throttler')
