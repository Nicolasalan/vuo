#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import imutils

class ColorFilter(object):
    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        os.system("v4l2-ctl -d /dev/video0 -c focus_auto=0 && v4l2-ctl -d /dev/video0 -c focus_absolute=0")
        os.system("v4l2-ctl -d /dev/video0 -c saturation=50")
        os.system("v4l2-ctl -d /dev/video0 -c brightness=100")
        os.system("v4l2-ctl -d /dev/video0 -c contrast=100")

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        #image = cv2.imread(example_path)
        #I resized the image so it can be easier to work with
        image = cv2.resize(cv_image,(640,480))

        #Once we read the image we need to change the color space to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        min = np.array([14, 20, 95])
        max = np.array([73, 77, 187])

        mask = cv2.inRange(hsv, min, max)
        output = cv2.bitwise_and(image, image, mask=mask)   
        kernel = np.ones((15,16),np.uint8)

        # aplicar blur para eliminar ruido
        blur = cv2.GaussianBlur(mask,(5,5),0)

        inErosion = cv2.erode(blur,kernel,iterations = 1)
        inDilation = cv2.dilate(inErosion,kernel,iterations = 1)

        cnts = cv2.findContours(inDilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # encontrar o maior centro de massa
        if len(cnts) != 0:
            # draw in blue the contours that were founded
            cv2.drawContours(output, cnts, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = max(cnts, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)

            # draw the biggest contour (c) in green
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)
            cx = x + w/2
            cy = y + h/2

            cv2.circle(inDilation, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.imshow("Result", np.hstack([image, output]))
    
        res_g = cv2.bitwise_and(image,image, mask=mask)

        cv2.waitKey(1)



def main():
    color_filter_object = ColorFilter()
    rospy.init_node('color_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()