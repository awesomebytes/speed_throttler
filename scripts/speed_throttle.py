#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

"""
Given an input Twist topic
limit its speed by using a LaserScan
topic.

Test it with: rosrun rqt_robot_steering rqt_robot_steering
"""


class Throttler(object):
    def __init__(self):

        rospy.loginfo("Initializing throttler...")
        self.scan_threshold = rospy.get_param('~scan_threshold', 0.5)
        self.twist_reducer = rospy.get_param('~twist_reducer', 0.001)
        self.last_scan = None
        self.scan_sub = rospy.Subscriber('~scan',
                                         LaserScan,
                                         self.scan_cb,
                                         queue_size=1)

        self.reduced_twist_pub = rospy.Publisher('~reduced_twist',
                                                 Twist,
                                                 queue_size=1)

        self.twist_sub = rospy.Subscriber('~input_twist',
                                          Twist,
                                          self.twist_cb,
                                          queue_size=1)
        rospy.loginfo("node initialised")

    def scan_cb(self, s):
        """
        :param LaserScan s: laser scan
        """
        #print "scan_cb"
        self.last_scan = s
        
        #print "calling"

    def twist_cb(self, t):
        """
        :param Twist t: twist command
        """
        # Check on the last scan message
        # if any point is under self.scan_threshold
        # if so, apply self.twist_reducer
        # and publish
        filteredArray = []
        for elem in self.last_scan.ranges:
            if elem > self.last_scan.range_min:
                #print elem, 
                filteredArray.append(elem)
        rospy.loginfo("filteredArray contains " + str(len(filteredArray)) + " elements")

        m = min(filteredArray)
        rospy.loginfo("minimal element is: " + str(m) + " (scan_threshold: " + str(self.scan_threshold))

        if m < self.scan_threshold:
            rospy.loginfo(" Previous speed is :")

            rospy.loginfo(t)
            t.linear.x *= self.twist_reducer
            t.linear.y *= self.twist_reducer
            t.linear.z *= self.twist_reducer

            t.angular.x *= self.twist_reducer
            t.angular.y *= self.twist_reducer
            t.angular.z *= self.twist_reducer
            
            rospy.loginfo("                  ")
            rospy.loginfo("Reduced Speed is :")
            rospy.loginfo(t)
            
        self.reduced_twist_pub.publish(t)

      




if __name__ == '__main__':
    rospy.init_node('speed_throttler')
    obj = Throttler()
    rospy.spin()