#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import threading
class safetyMech: 
  def __init__(self):	
    self.node_name = "SafetyMech"
    rospy.Subscriber('/scan', LaserScan, self.laser_callback)
    #rospy.Subscriber('/drive',AckermannDriveStamped,self.drive_callback)
    self.publisher = rospy.Publisher('/vesc/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)
    self.reg_drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation',AckermannDriveStamped,queue_size=1)
    self.unsafe = False
    self.count = 0
    self.lastSteerAngle = 0
    self.drive_msg = AckermannDriveStamped()
    self.emergency_lock = threading.Lock()

  """def drive_callback(self,msg):
	if self.emergency_lock.locked():
	    pass

	else:
	    msg.drive.speed = min(msg.drive.speed,2.0)
	    self.lastSteerAngle = msg.drive.steering_angle
	    self.reg_drive.publish(msg)	
"""
  def laser_callback(self, msg):
    #print self.unsafe
    ranges = map(lambda x: int(x<0.5), msg.ranges)
    danger_ranges = ranges[360:711]
    if sum(danger_ranges) > 3:
      self.unsafe = True
    else:
      self.unsafe = False
    '''
    #print ranges[360:711]
    for i in range(360, 711):
      #print sum(ranges[i:i+10])
      if ranges[i] == 1:
        self.unsafe = True
    '''
    print self.unsafe
    print self.count
    if self.unsafe == True:
      if not (self.emergency_lock.locked()):
        self.emergency_lock.acquire(False)
      if self.count > 4:
      	self.drive_msg.drive.steering_angle = self.lastSteerAngle
      	self.drive_msg.drive.speed = -1.0
      else:
        self.drive_msg.drive.steering_angle = self.lastSteerAngle
        self.drive_msg.drive.speed = 0.0
      self.counter += 1
      self.drive_msg.header.stamp = rospy.Time.now()
      self.publisher.publish(self.drive_msg)
      #rospy.Rate(8).sleep()
    else:
      self.counter = 0
      if self.emergency_lock.locked():
	self.emergency_lock.release()
'''    
  def publish(self):
    while not rospy.is_shutdown():
      	while self.unsafe:
            print self.unsafe
	    self.counter += 1
            self.publisher.publish(self.drive_msg)
            rospy.Rate(8).sleep()
'''		 
if __name__ == "__main__":
    rospy.init_node("SafetyMech")
    e = safetyMech()
    rospy.spin()
