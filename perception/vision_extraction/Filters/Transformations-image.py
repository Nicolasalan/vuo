#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Para transformacao em video o unica transformacao seria o cv_image ao invez de img

class Transformations(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # Selecionamos bgr8 porque é a codificação OpenCV por padrão
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        example_path = '{caminho da imagem}.png'
            
        img = cv2.imread(example_path,0)
        img = cv2.resize(img,(250,250))

        # Define um kernel para a erosão 
        kernel_a = np.ones((5,5),np.uint8)
        erosion = cv2.erode(img,kernel_a,iterations = 1)

        # Define um kernel para a dilatação
        kernel_b = np.ones((3,3),np.uint8)
        dilation = cv2.dilate(img,kernel_b,iterations = 1)

        # Define um kernel para a abertura
        kernel_c = np.ones((7,7),np.uint8)
        opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel_c)

        # Define um kernel para o fechamento
        kernel_d = np.ones((7,7),np.uint8)
        closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel_d)

        cv2.imshow('Original',img)
        cv2.imshow('Erosion',erosion)
        cv2.imshow('Dilation',dilation)
        cv2.imshow('Opening',opening)
        cv2.imshow('Closing',closing)

        cv2.waitKey(1)

def main():
    transformations_object = Transformations()
    rospy.init_node('transformations_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()