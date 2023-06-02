#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # importacao
import cv2
import numpy as np


class MostrarImagem(object): # definicao da classe

    def __init__(self): # inicializacao

          self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback) # topico
          self.bridge_object = CvBridge() # OpenCV

    def camera_callback(self,data):
          try:
          # Selecionamos bgr8 porque é a codificação OpenCV por padrão
               cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") # conversao
          except CvBridgeError as e: # caso mostre um erro em encontrar a imagem 
               print(e)

          #Importe a biblioteca numpy que ajudará em algumas operações de matriz
          #image = cv2.imread(cv_image)

          #Redimensionei a imagem para que seja mais fácil trabalhar com
          image = cv2.resize(cv_image,(300,300))

          # Uma vez que lemos a imagem, precisamos mudar o espaço de cores para HSV
          hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

          #Hsv limites são definidos
          #aqui é onde você define a gama de cores que procura
          #cada valor do vetor corresponde aos valores H, S e V respectivamente
          min_green = np.array([40,220,220]) # 
          max_green = np.array([60,255,255])

          min_red = np.array([0,220,220])
          max_red = np.array([10,255,255])

          min_blue = np.array([100,220,220])
          max_blue = np.array([120,255,255])


          #Esta é a detecção de cor real
          #Aqui vamos criar uma máscara que contém apenas as cores definidas em seus limites
          #Esta máscara tem apenas uma dimensão, então é preto e branco
          mask_g = cv2.inRange(hsv, min_green, max_green)
          mask_r = cv2.inRange(hsv, min_red, max_red)
          mask_b = cv2.inRange(hsv, min_blue, max_blue)

          # Usamos a máscara com a imagem original para obter a imagem colorida pós-processada
          res_b = cv2.bitwise_and(image, image, mask= mask_b)
          res_g = cv2.bitwise_and(image,image, mask= mask_g)
          res_r = cv2.bitwise_and(image,image, mask= mask_r)

          cv2.imshow('real_image',image)
          cv2.imshow('image 1',res_g)
          cv2.imshow('image 2',res_r)
          cv2.imshow('image 3',res_b)
 
          cv2.waitKey(1)



def main():
     Imagem_objeto = MostrarImagem()
     rospy.init_node('load_image_node', anonymous=True)
     try:
          rospy.spin()
     except KeyboardInterrupt:
          print("Desligando")
     cv2.destroyAllWindows()

if __name__ == '__main__':
     main()