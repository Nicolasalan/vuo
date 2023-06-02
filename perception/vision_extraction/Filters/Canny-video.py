#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# O filtro Canny é um detector de bordas
class CannyFilter(object):

    def __init__(self):
    
        #  topico onde se localiza a camera
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback) 
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") # captura do video
        except CvBridgeError as e:
            print(e)
        
        img = cv2.resize(cv_image,(450,350)) # redimensionar o tamanho do video

        minV = 30
        maxV = 100

        edges = cv2.Canny(img,minV,maxV) # Filtro canny 
        cv2.imshow('Original',img) # Mostrar imagem original
        cv2.imshow('Edges',edges) # Mostrar imagem com filtro

        cv2.waitKey(1)


# inicializacao do node
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