#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image as img
from std_msgs.msg import String
from cv_bridge import CvBridge
import PIL
from PIL import ImageFont
from PIL import Image
from PIL import ImageDraw
import random
import time
hrange = [0,180]
srange = [0,256]
ranges = hrange+srange
class saveColor:
        def __init__(self):
                self.bridge = CvBridge()
                self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", img, self.camCallback)
                self.img_pub = rospy.Publisher("/exploring_challenge", String, queue_size=10)
                self.index = 1
                self.racecar = cv2.imread('image01.png')
                self.racecar = cv2.cvtColor(self.racecar, cv2.COLOR_BGR2HSV)
                self.racecar = cv2.calcHist(self.racecar,[0,1],None,[180,256],ranges)
                self.ari = cv2.imread('image02.jpg')
                self.ari = cv2.cvtColor(self.ari, cv2.COLOR_BGR2HSV)
                self.ari = cv2.calcHist(self.ari,[0,1],None,[180,256],ranges)
                self.sertac = cv2.imread('image00.jpg')
                self.sertac = cv2.cvtColor(self.sertac, cv2.COLOR_BGR2HSV)
                self.sertac = cv2.calcHist(self.sertac,[0,1],None,[180,256],ranges)
                self.cat = cv2.imread('image03.jpg')
                self.cat = cv2.cvtColor(self.cat, cv2.COLOR_BGR2HSV)
                self.cat = cv2.calcHist(self.cat,[0,1],None,[180,256],ranges)
        def camCallback(self,msg):
                rospy.loginfo("Image recieved! Processing...")
                img_data = self.bridge.imgmsg_to_cv2(msg)
                self.processImg(img_data)
                time.sleep(1)
        #processed_img = self.bridge.cv2_to_imgmsg(processed_img_cv2, "bgr8")
        def processImg(self,img):
                hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
                contourList = []
                maskG = cv2.inRange(hsv, np.array([35,100,100]), np.array([70, 255, 255]))
                maskR1 = cv2.inRange(hsv, np.array([0,150,100]), np.array([10, 255, 200]))
                maskR2 = cv2.inRange(hsv, np.array([160,150,100]), np.array([180,255,200]))
                maskR = maskR1 + maskR2
                maskB = cv2.inRange(hsv, np.array([100,100,100]), np.array([130, 255, 255]))
                maskY = cv2.inRange(hsv, np.array([23,100,160]), np.array([30, 255, 255]))
                maskP1 = cv2.inRange(hsv, np.array([0,50,230]), np.array([10, 150, 255]))
                maskP2 = cv2.inRange(hsv, np.array([150,50,230]), np.array([180,150,255]))
                maskP = maskP1+maskP2
                contoursG = cv2.findContours(maskG, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contoursR = cv2.findContours(maskR, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contoursB = cv2.findContours(maskB, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contoursY = cv2.findContours(maskY, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contoursP = cv2.findContours(maskP, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                self.contourAppend(contourList,contoursG,"green")
                self.contourAppend(contourList,contoursR,"red")
                self.contourAppend(contourList,contoursB,"blue")
                self.contourAppend(contourList,contoursY,"yellow")
                self.contourAppend(contourList,contoursP,"pink")
                if len(contourList)>0:
                        biggest = self.findBiggest(contourList)
                else:
                        biggest = None
                if biggest != None:
                        print(biggest.text)
                        #print(biggest.contour)
                        cv2.drawContours(img, biggest.contour, -1, (0, 255, 0), 3)
                        #cv2.imshow("oooo",img)
                        #cv2.waitKey(0)
                        if biggest.text != "pink":
                                self.saveImg(img,biggest.text)
                        else:
                                x,y,w,h = cv2.boundingRect(biggest.contour)
                                sliced = hsv[x:x+w,y:y+h,:]
                                hsvTest = cv2.calcHist(sliced,[0,1],None,[180,256],ranges)
                                racecarVal = cv2.compareHist(hsvTest,self.racecar,cv2.cv.CV_COMP_CORREL)
                                ariVal = cv2.compareHist(hsvTest,self.ari,cv2.cv.CV_COMP_CORREL)
                                sertacVal = cv2.compareHist(hsvTest,self.sertac,cv2.cv.CV_COMP_CORREL)
                                catVal = cv2.compareHist(hsvTest,self.cat,cv2.cv.CV_COMP_CORREL)
                                maxVal = max(racecarVal,ariVal,sertacVal,catVal)
                                if maxVal == racecarVal:
                                        self.saveImg(img,"racecar")
                                elif maxVal == ariVal:
                                        self.saveImg(img,"ari")
                                elif maxVal == sertacVal:
                                        self.saveImg(img,"proffesor karaman")
                                else:
                                        self.saveImg(img,"cat")
        def saveImg(self,img,text):
                cv2.imwrite("troll.jpeg",img)
                pic = Image.open("troll.jpeg")
                font = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf",25)
                draw = ImageDraw.Draw(pic)
                draw.text((0, 0),text,(255,255,0),font=font)
                rand = self.index
                self.index = self.index + 1
                fileName = str(rand)+".jpeg"
                pic.save(fileName)
                self.img_pub.publish(text)
        def findBiggest(self,contourList):
                result = contourList[0]
                for x in contourList:
                        if cv2.contourArea(x.contour)>cv2.contourArea(result.contour):
                                result = x
                if cv2.contourArea(result.contour)>0:
                        return result
                else:
                        return None
        def contourAppend(self,contourList,contour,color):
                for x in contour[0]:
                        appendedStuff = contours(x,color)
                        contourList.append(appendedStuff)

        def pinkImageDetection(self, image):
            x,y,w,h = cv2.boundingRect(image.contour)
            sliced = hsv[x:x+w,y:y+h,:]
            hsvTest = cv2.calcHist(sliced,[0,1],None,[180,256],ranges)
            racecarVal = cv2.compareHist(hsvTest,self.racecar,cv2.cv.CV_COMP_CORREL)
            ariVal = cv2.compareHist(hsvTest,self.ari,cv2.cv.CV_COMP_CORREL)
            sertacVal = cv2.compareHist(hsvTest,self.sertac,cv2.cv.CV_COMP_CORREL)
            catVal = cv2.compareHist(hsvTest,self.cat,cv2.cv.CV_COMP_CORREL)
            maxVal = max(racecarVal,ariVal,sertacVal,catVal)

            if maxVal == racecarVal:
                print "racecar"
                    #self.saveImg(img,"racecar")
            elif maxVal == ariVal:
                print "ari"
                    #self.saveImg(img,"racecar")
            elif maxVal == sertacVal:
                print "karaman"
                    #self.saveImg(img,"racecar")
            else:
                print "cat"
                    #self.saveImg(img,"racecar")


class contours:
        def __init__(self,contour,text):
                self.contour = contour
                self.text = text
if __name__ == "__main__":
        rospy.init_node("save_color")
        node = saveColor()
        rospy.spin()
