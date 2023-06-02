#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

# DETECTAR FACE E SALVAR
class LoadFace(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
          try:
               # We select bgr8 because its the OpenCV encoding by default
               cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
          except CvBridgeError as e:
               print(e)

          face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3/haar_cascades/frontalface.xml')
          eyes_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3/haar_cascades/eye.xml')


          img = cv_image

          img = cv2.resize(img,(350,550))

          # convertemos a imagem em tons de cinza
          gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

          # O fator de escala determina o quanto a imagem original será reduzida em cada escala
          ScaleFactor = 1.2

          # determinará o número de vizinhos para um valor mais alto
          minNeighbors = 3
          
          # Aplicamos as cascatas às nossas imagens em tons de cinza
          faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)

          # extrair as coordenadas das detecções
          for (x,y,w,h) in faces:
               # Para cada coordenada, desenharemos um retângulo na imagem original
               cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)  
               # Este será um corte da área de interesse
               roi_gray = gray[y:y+h, x:x+w]
               roi_color = img[y:y+h, x:x+w]
               eyes = eyes_cascade.detectMultiScale(roi_gray)
               for (ex,ey,ew,eh) in eyes:
                    cv2.rectangle(roi_color, (ex,ey),(ex+ew,ey+eh),(0,255,0),2)
                    cv2.imwrite('Caminhi para salvar imagem aqui.jpg',img)

          
          cv2.imshow('Image',img)

          cv2.waitKey(1)
          



def main():
    Face_detection = LoadFace()
    rospy.init_node('Face_detection_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()