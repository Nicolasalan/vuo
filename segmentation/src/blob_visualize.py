#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import imutils

class ColorVisualize(object):
    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

        # parametros para calibrar camera (hardware) como foco, brilho e ajuste 
        os.system("v4l2-ctl -d /dev/video0 -c focus_auto=0 && v4l2-ctl -d /dev/video0 -c focus_absolute=0")
        os.system("v4l2-ctl -d /dev/video0 -c saturation=50")
        os.system("v4l2-ctl -d /dev/video0 -c brightness=100")

    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Redimensionei a imagem para facilitar o trabalho
        image = cv2.resize(cv_image,(640,480))

        # Depois de ler a imagem, precisamos alterar o espaço de cores para HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Limites de Hsv são definidos
        # aqui é onde você define o intervalo da cor que procura
        # cada valor do vetor corresponde aos valores H,S e V respectivamente
        min_green = np.array([0, 0, 0])
        max_green = np.array([180, 255, 78])

        # Esta é a detecção de cor atual
        # Aqui vamos criar uma máscara que contém apenas as cores definidas em seus limites
        # Esta máscara tem apenas uma dimensão, então é preto e branco }
        mask = cv2.inRange(hsv, min_green, max_green)
        output = cv2.bitwise_and(image, image, mask=mask)
        kernel = np.ones((15,16),np.uint8)
        inErosion = cv2.erode(mask,kernel,iterations = 1)
        inDilation = cv2.dilate(inErosion,kernel,iterations = 1)

        cnts = cv2.findContours(inDilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # encontrar o maior centro de massa
        if len(cnts) != 0:
            # desenha em azul os contornos que foram fundados
            cv2.drawContours(output, cnts, -1, 255, 3)

            # encontre o maior contador (c) pela área
            c = max(cnts, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)

            # desenha o maior contorno (c) em verde
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)
            # pegar o centro do retangulo
            cx = x + w/2
            cy = y + h/2

            cv2.circle(inDilation, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.imshow("Result", np.hstack([image, output]))
        

        res_g = cv2.bitwise_and(image,image, mask=mask)

        cv2.waitKey(1)



def main():
    color_filter_object = ColorVisualize()
    rospy.init_node('color_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
