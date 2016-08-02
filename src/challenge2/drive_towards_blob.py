#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32MultiArray, Int32
from final77.msg import blob as BlobMsg

import math

STATE_FOLLOW_BLOB = 0
STATE_NUDGE       = 1
STATE_FOLLOW_WALL = 2

TARGET_NONE = 0
TARGET_GREEN = 1
TARGET_RED = 2

class Control:

    def __init__(self):
        rospy.Subscriber("/detection", BlobMsg, self.detection_recieved)
        
        rospy.Subscriber("/wallfollower_left", AckermannDriveStamped, self.wallfollower_left)
        rospy.Subscriber("/wallfollower_right", AckermannDriveStamped, self.wallfollower_right)
 
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

        #where we want the centroid to be in relation to the screen
        self.x_des = 640
        #desired area of object on screen
        self.area_des = 7000
        #how accurate the centroid is from our current position
        self.centroid_threshhold = 20
        #how accurate the area is from the current area of the object self.area_threshhold = 80
        #initial steering angle
        self.steering_angle = 0.0
        #initial speed
        self.speed = 1.3
        #p constant for pid controll
        self.follow_blob_K_p = -0.0005

        self.follow_wall_K_p = 0.5
        self.follow_wall_desired = 0.65

        # temp
        #self.direction = 'right'
        self.state = STATE_FOLLOW_BLOB

    #calculate what the steering angle should be based on how far the centroid of the object is from the middle of the screen
    def angle_control(self, centroid_coor):
        centroid_errorx = centroid_coor - self.x_des
        #if the difference in centroid is small enough, don't have to change steering angle
        if abs(centroid_errorx) <= self.centroid_threshhold:
            steering_angle = 0
        #if not, use proportional control
        else:
            p = self.follow_blob_K_p * centroid_errorx
            steering_angle = p
        return steering_angle

    #callback function for recieving msgs from detection
    def detection_recieved(self, msg):
        if msg.target == 0:
            return

        if self.state == STATE_FOLLOW_BLOB:
            speed = 200 / math.sqrt(msg.area)
            print(speed)
            self.drive(
                #pass in actual area, which is the first arg of the message, to change the speed
                2,
                #pass in the x of the centroid of the obj to change the steering angle
                self.angle_control(msg.x)
            )
            if msg.area > self.area_des:
                self.state = STATE_NUDGE
                self.nudge_iteration = 2 if msg.target == 1 else 0
                self.direction = 'right' if msg.target == 1 else 'left'
                self.nudge_timer = rospy.Timer(rospy.Duration(.2), self.nudge_callback)


    def wallfollower_left(self, msg):
        if self.state == STATE_FOLLOW_WALL and self.direction == 'left':
            self.drive_pub.publish(msg)

    def wallfollower_right(self, msg):
        if self.state == STATE_FOLLOW_WALL and self.direction == 'right':
            self.drive_pub.publish(msg)
            
    def nudge_callback(self, _):
        if self.direction == 'left':
            self.drive(1.5, 0.40)
        elif self.direction == 'right':
            self.drive(1.0, -0.35)

        print("doing nudge")
        
        self.nudge_iteration += 1
        if self.nudge_iteration > 8:
            print("ENTERING WALL FOLLOWING")
            self.nudge_timer.shutdown()
            self.state = STATE_FOLLOW_WALL

    #callback function for driving
    def drive(self, speed, steering_angle):
        out = AckermannDriveStamped()
        out.drive.speed = speed
        out.drive.steering_angle = steering_angle

        self.drive_pub.publish(out)

if __name__=="__main__":
    rospy.init_node("Control")
    node = Control()
    rospy.spin()
