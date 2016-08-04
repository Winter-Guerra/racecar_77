#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import os
import shutil
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String, Bool
from cv_bridge import CvBridge, CvBridgeError
import threading


class LightTracker:
    def __init__(self, debugging):
        self.node_name = "LightTracker"
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_notification = rospy.Publisher("/turnRight", Bool, queue_size=10)
        self.image_pub = rospy.Publisher("~Echo", Image, queue_size =1)
        self.notification = Bool()
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.debugging = debugging
    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def detection(self, img):
        self.notification = False
        #print "detection"
        bounds = [["red", [0, 100, 50], [15,255,255]]]

        for i in bounds:
            lower = np.array(i[1])
            upper = np.array(i[2])
            color = i[0]
        ret = self.detect_color_blob(img, lower, upper, color)
        print ret
        #print lower
        color_code = bounds.index(i)+1

        if ret == None:
            cx = 0
            cy = 0
            area = 0
            color_code = 0
        else:
            cx, cy, area = ret

        #publish here!
        self.pub_notification.publish(self.notification)
        

    def detect_color_blob(self, img, lower, upper, color):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        mask = cv2.erode(mask, (3,3), iterations=1)

        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        sorted_contours = sorted(contours, key = lambda c: cv2.contourArea(c), reverse=True)
        
        if len(sorted_contours) < 1:
            return None

        c = sorted_contours[0]

        area = cv2.contourArea(c)
        if area < 2000: # minimum area threshold'
            print lower
            print area
            print "too small"
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
	try:
        	cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
	except:
		cx,cy = 0 , 0
        self.notification = True
        print self.notification
        now = rospy.Time.now()

        if self.lock.acquire(False):
                self.saveImage(img, now)

        if self.debugging:
            cv2.circle(img, (cx, cy), 10, (255, 255, 255), -1)

        approx_area = cv2.contourArea(approx)
        ros_img = self.bridge.cv2_to_imgmsg(img)
        self.image_pub.publish(ros_img)
        return (cx, cy, approx_area)

    def processImage(self, image_msg):
        if not self.lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        self.detection(image_cv )

        if self.debugging:
            try:
                self.image_pub.publish(\
                        self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
            except CvBridgeError as e:
                print(e)
        self.lock.release()


if __name__=="__main__":
    rospy.init_node('LightTracker')
    e = LightTracker(True)
    rospy.spin()

