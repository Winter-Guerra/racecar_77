#!/usr/bin/env python

'''
Created by Winter Guerra <winterg@mit.edu> on July 2016.
Modified by Team 77 (Clara Li, Jack Messina, Aman Jha, Justin Yu, Caleb Trotz) in July/August 2016
Modified again by Winter Guerra on 9/12
'''

# import main ROS python library
import rospy

# import message types
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped

# Import numpy for sanity
from rospy.numpy_msg import numpy_msg
import numpy as np
import math

# simple class to contain the node's variables and code
class PotentialField:
    # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self):
        # initialize potential field variables
        self.charge_laser_particle = 0.055
        self.charge_forward_boost = 25.0
        self.boost_distance = 0.5
	self.certainty = 0
        self.p_speed = 0.05       
        self.p_steering = 1.0
	self.p_d = 0.5
        self.turn_right = False
	self.last_y = 0.0
        self.turn_x_component = np.zeros(1)
        self.turn_y_component = np.zeros(1)
        # subscribe to turning signal
        rospy.Subscriber("/turnRight", Bool, self.bool_back)
        # subscribe to laserscans. Force output message data to be in numpy arrays.
        rospy.Subscriber("/scan", numpy_msg(LaserScan), self.scan_callback)
        self.angle_min = 0
        self.angle_increment = 0
        # output a pose of where we want to go
        self.pub_goal = rospy.Publisher("~potentialFieldGoal", PointStamped, queue_size=1)
        self.pub_nav = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.speedHist = 1.0
        
    def bool_back(self, msg):
        self.turn_right = msg.data
	self.certainty = 19.0*self.certainty/20.0 + self.turn_right/20.0
	self.turn_right = (self.certainty > 0.7)
        print "Should I turn: {}".format(self.certainty)
        
    
    def scan_callback(self, msg):
        # Debug
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        #print("Starting increment {} increment {}".format(msg.angle_min, msg.angle_increment))
        ranges = msg.ranges[180:901]
        # Create potential gradients for middle 180 degrees of laser scan particles
        scan_rad_angles = ( (msg.angle_increment * np.arange(1081, dtype=float)) + msg.angle_min )
        scan_rad_angles = scan_rad_angles[180:901]

        scan_x_unit_vectors = -np.cos(scan_rad_angles)
        scan_y_unit_vectors = -np.sin(scan_rad_angles)

        

        # Add the potential for the point behind the robot (to give it a kick)
        kick_x_component = np.ones(1) * self.charge_forward_boost / self.boost_distance**2.0
	kick_y_component = np.zeros(1)

	# Make the robot turn
	kick_y_component += (20.0)

	# Make the robot stop on red
	should_brake = not self.turn_right
    
	scan_x_components = (self.charge_laser_particle * scan_x_unit_vectors) / np.square(ranges)
        scan_y_components = (self.charge_laser_particle * scan_y_unit_vectors) / np.square(ranges)
        # Add together the gradients to create a global gradient showing the robot which direction to travel in
        total_x_component = np.sum(scan_x_components) + kick_x_component
        total_y_component = np.sum(scan_y_components) + kick_y_component

	d_val = self.last_y - total_y_component

        #print self.p_steering * np.sign(total_x_component) * math.atan2(total_y_component+self.p_d*d_val, total_x_component)

        # Transform this gradient vector into a PoseStamped object
        visualizer_msg = PointStamped()
        visualizer_msg.header.frame_id = 'base_link'
        visualizer_msg.point.x = total_x_component
        visualizer_msg.point.y = total_y_component

        # Publish this goal so that we can see it in RVIZ
        self.pub_goal.publish(visualizer_msg)

        # Now, create a steering command to send to the vesc.
        command_msg = AckermannDriveStamped()
        command_msg.drive.steering_angle = (self.p_steering * np.sign(total_x_component) * math.atan2(total_y_component+self.p_d*d_val, total_x_component))
        command_msg.drive.speed = (self.p_speed * np.sign(total_x_component) * math.sqrt(total_x_component**2 + total_y_component**2)) * should_brake

        self.pub_nav.publish(command_msg)
	# Log the last y, but slow down the derivative
	self.last_y = (self.last_y/2.0) + (total_y_component/2.0)

    def kickOut(self, speed):
        self.speedHist = (self.speedHist * .75) + (abs(speed) * .25)
        #print self.speedHist
        if self.speedHist < 0.2:
            return -1.0
        else:
            return speed



if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("potential_field_node")

    node = PotentialField()

    # enter the ROS main loop
    rospy.spin()
