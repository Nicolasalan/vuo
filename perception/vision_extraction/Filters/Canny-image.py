#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# DETECTAR UMA AMPLA MARGEM DE BORDAS NA IMAGEM 
class CannyFilter(object):

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
        img = cv2.imread(example_path)
        img = cv2.resize(img,(450,350))

        # O detector inteligente usa dois parâmetros além da imagem:
        # O gradiente de intensidade mínimo e máximo
        minV = 30
        maxV = 100

        edges = cv2.Canny(img,minV,maxV)
        cv2.imshow('Original',img)
        cv2.imshow('Edges',edges)

        cv2.waitKey(1)



def main():
    canny_filter_object = CannyFilter()
    rospy.init_node('canny_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()