#!/usr/bin/env python
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from final77.msg import blob as BlobMsg

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

        self.pub_detection = rospy.Publisher("/detection", BlobMsg, queue_size=10)

        self.debugging = debugging

        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def detection(self, img):
        green_lower = np.array([54, 30, 60])
        green_upper = np.array([72, 255, 255])

        color = "GREEN"
        ret = self.detect_color_blob(img, green_lower, green_upper, color)
        color_code = 1
        
        if ret == None:
            red_lower = np.array([0, 160, 130])
            red_upper = np.array([15, 255, 255])
            color = "RED"
            ret = self.detect_color_blob(img, red_lower, red_upper, color)
            color_code = 2
            

        if ret == None:
            cx = 0
            cy = 0
            area = 0
            color_code = 0
        else:
            cx, cy, area = ret

        msg = BlobMsg()
        msg.area = area
        msg.x = cx
        msg.target = color_code

        print(msg)
        self.pub_detection.publish(msg)
        # publish message

    def detect_color_blob(self, img, lower, upper, color):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        mask = cv2.erode(mask, (3,3), iterations=1)

        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if self.debugging:
            cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

        sorted_contours = sorted(contours, key = lambda c: cv2.contourArea(c), reverse=True)

        if len(sorted_contours) < 1:
            return None

        c = sorted_contours[0]

        area = cv2.contourArea(c)
        if area < 400: # minimum area threshold
            return None

        perim = cv2.arcLength(c, True) # perimeter
        approx = cv2.approxPolyDP(c, 0.05 * perim, True)

        if len(approx) != 4:
            return None


        if self.debugging:
            cv2.drawContours(img, [c], -1, (255, 0, 0), 3) 
            cv2.drawContours(img, [approx], -1, (0, 255, 0), 5) 

            coord = (approx[0][0][0], approx[0][0][1])
            cv2.putText(img, color, coord, cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255),  2)

        M = cv2.moments(approx)
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        if self.debugging:
            cv2.circle(img, (cx, cy), 10, (255, 255, 255), -1)

        approx_area = cv2.contourArea(approx)

        return (cx, cy, approx_area)


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        self.detection(image_cv)
        
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

