#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math
from PID import PIDController

PID_KP = 0.9
PID_KP_countclock = 0.0035
PID_KI = 0.0
PID_KD = 0.01
SLICE_LEN = 50
class twoWallFollow:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
        self.publisher = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        self.rangeStarted = False
        self.drive_msg = AckermannDriveStamped()
        self.pid = PIDController(rospy.Time.now(),  PID_KP,PID_KI,PID_KD)
        self.publish()

    def laser_callback(self, msg):
        ranges = map(lambda x: int(x>1.5),  msg.ranges)
	ranges= ranges[180:901]
        self.rangeStarted = False
        #print ranges
        currStart = 0
        currEnd = 0
        currLen = 0
        maxLen = 0
        maxEnd = 0
        maxStart = 0
        for i in range(len(ranges)):
            if ranges[i]:
                #print "empty"
                if self.rangeStarted:
                    pass
                else:
                    currStart = i
                    #print currStart
                    #print "started Range at" + str(currStart)
                    self.rangeStarted = True
            else:
                #print self.rangeStarted
                if self.rangeStarted:
                    currEnd = i
                    #print currEnd
                    currLen = currEnd - currStart
                    #print currLen
                    if currLen > maxLen:
                        print "this"
                        maxLen = currLen
                        maxStart = currStart
                        maxEnd = currEnd
                    self.rangeStarted = False
                else:
                    continue
        targetPoint = (maxEnd + maxStart)/2 + 180 if maxEnd != 0 else 540 
        print targetPoint
	print maxLen
        error = (targetPoint * msg.angle_increment) + msg.angle_min
	
        steer_angle = self.pid.update(error,  rospy.Time.now())
	print steer_angle
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 2.0 if maxLen >= 115 else -1.0
        self.drive_msg.drive.steering_angle = steer_angle

    def publish(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.drive_msg)
            rospy.Rate(8).sleep()

if __name__ == "__main__":
    rospy.init_node("TwoWallFollow")
    e = twoWallFollow()
    rospy.spin()
