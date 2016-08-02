#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import os
import shutil

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
#from racecar_localization.msg import blob as BlobMsg

from cv_bridge import CvBridge, CvBridgeError
import threading


class ColorTracker:
    def __init__(self, debugging):
        self.node_name = "ColorTracker"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)

        #self.pub_detection = rospy.Publisher("/detection", BlobMsg, queue_size=10)
        self.pub_notification = rospy.Publisher("/exploring_challenge", String, queue_size=10)
        self.notification = String()

        self.image_count = 0

        self.debugging = debugging

        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))
        
        self.dirname = '/home/racecar/challenge_photos/'
        
	self.lock = threading.Lock()

	if os.path.exists(self.dirname):
		shutil.rmtree('/home/racecar/challenge_photos/')      
		print "folder removed"      
	os.makedirs(self.dirname)
        print "new folder created"
        

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def detection(self, img):

        bounds = [["green", [50, 100, 100], [77, 255, 255]], ["red", [0, 130, 130], [10, 255, 255]], ["red", [170, 130, 130], [180,255,255]], ["blue", [100, 60, 80], [130, 255, 255]], ["yellow", [14, 100, 136], [29, 255, 255]], ["racecar", [0, 0, 0], [0, 0, 0]], ["ari", [0, 0, 0], [0, 0, 0]], ["professor karaman", [0, 0, 0], [0, 0, 0]], ["cat", [0, 0, 0], [0, 0, 0]]]
        
        for i in bounds:
            lower = np.array(i[1])
            upper = np.array(i[2])
            color = i[0]

            ret = self.detect_color_blob(img, lower, upper, color)
            color_code = bounds.index(i)+1

        if ret == None:
            cx = 0
            cy = 0
            area = 0
            color_code = 0
        else:
            cx, cy, area = ret

        
        #msg = BlobMsg()
        #msg.area = area
        #msg.x = cx
        #msg.target = color_code

        #print(msg)
        #self.pub_detection.publish(msg)
        # publish message
        self.pub_notification.publish(self.notification)

    def detect_color_blob(self, img, lower, upper, color):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        mask = cv2.erode(mask, (3,3), iterations=1)

        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #if self.debugging:
            #cv2.drawContours(img, contours, -1, (0, 0, 255), 2)
                                  
        sorted_contours = sorted(contours, key = lambda c: cv2.contourArea(c), reverse=True)

        if len(sorted_contours) < 1:
            return None

        c = sorted_contours[0]

        area = cv2.contourArea(c)
        if area < 1000: # minimum area threshold
            return None

        perim = cv2.arcLength(c, True) # perimeter
        approx = cv2.approxPolyDP(c, 0.05 * perim, True)

        if len(approx) != 4:
            return None


        if self.debugging:
            #cv2.drawContours(img, [c], -1, (255, 0, 0), 3) 
            cv2.drawContours(img, [approx], -1, (0, 255, 0), 5) 

            coord = (approx[0][0][0], approx[0][0][1])
            cv2.putText(img, color, coord, cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255),  2)

        M = cv2.moments(approx)
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        self.notification = "I see "+color
        #self.photo_iter = 0
        #self.photo_timer = rospy.Timer(rospy.Duration(0.5), self.saveImage(img))
        now = rospy.Time.now()
	if self.lock.acquire(False):
            self.saveImage(img, now)

        if self.debugging:
            cv2.circle(img, (cx, cy), 10, (255, 255, 255), -1)

        approx_area = cv2.contourArea(approx)

        return (cx, cy, approx_area)

    def saveImage(self, img, now):
        #self.photo_iter += 1
        #if self.photo_iter > 5:
        # if rospy.Time.now() - now < rospy.Duration(3): 
        path = str(self.image_count)+".png"
        print path
	
        cv2.imwrite(os.path.join(self.dirname, path), img)
        self.image_count += 1
	rospy.sleep(1)
	self.lock.release()
	# create a ROS timer for the amount of time which has a cbfunc
	# in that cbfunc release the lock
	   
        #self.photo_timer.shutdown()
        

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        self.detection(image_cv )
        
        if self.debugging:
            try:
                self.pub_image.publish(\
                        self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
            except CvBridgeError as e:
                print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('ColorTracker')
    e = ColorTracker(True)
    rospy.spin()

