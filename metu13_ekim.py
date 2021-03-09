from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import os
import dronekit
from dronekit import connect ,VehicleMode, LocationGlobalRelative
import datetime

#mavlink connection
vehicle = connect('127.0.0.1:14550', wait_ready=True)

#camera settings
camera = PiCamera()
camera.resolution =(1088 ,720)
#camera.resolution =( 1920 , 1088)
camera.framerate= 5
rawCapture = PiRGBArray(camera, size=(1088,720))
camera.awb_mode = 'tungsten'
camera.exposure_mode = 'antishake'
camera.iso = 200

#hsv filtre
lower_red=(128,80,90)
upper_red=(255,255,255)

#kameranin acilis zamani
time.sleep(0.1)

#folder time stamp
timestamp=time.strftime('%a-%H-%M-%S')

#saving image number
deger = 0


os.makedirs("/home/pi/ATES/FOTO/Deneme-metu"+str(timestamp))

for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
        #a=time.time()
        image=frame.array
        print ("calisiyor")
        #b=time.time()
        hsv_copy=image.copy()
        org=image.copy()
        hsv_copy=cv2.cvtColor(hsv_copy,cv2.COLOR_BGR2HSV)
        maske=cv2.inRange(hsv_copy,lower_red,upper_red)
        #maske=cv2.erode(maske,None,iterations=3) maske=cv2.dilate(maske,None,iterations=2)
        cnts=cv2.findContours(maske.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center=None
        if len(cnts)>0:
                c=max(cnts,key=cv2.contourArea)
                ((x,y),radius)=cv2.minEnclosingCircle(c)
                M=cv2.moments(c)
                if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                else:
                        cX, cY = 0, 0
                if radius>1:
                        cv2.circle(hsv_copy,(int(x),int(y)),int(radius),(0,255,255),2)
                        cv2.circle(hsv_copy,center,5,(0,0,255),-1)
                        cv2.circle(org,(int(x),int(y)),int(radius),(0,255,255),2)
                        cv2.putText(hsv_copy,str(x)+"-"+str(y),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,225,255),1)
                        cv2.putText(org,str(x)+"-"+str(y),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,225,255),1)
                        cv2.putText(org,str(vehicle.location.global_relative_frame.lat),(20,20),cv2.FONT_HERSHEY_SIMPLEX,1,(0,225,0),1)
                        cv2.putText(org,str(vehicle.location.global_relative_frame.lon),(20,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,225,0),1)
                        cv2.putText(org,str(vehicle.location.global_relative_frame.alt),(20,60),cv2.FONT_HERSHEY_SIMPLEX,1,(0,225,0),1)
                        name="metu"+str(deger)+".png"
                        cv2.imwrite(os.path.join(("/home/pi/ATES/FOTO/Deneme-metu"+str(timestamp)),"metu"+str(deger)+".png") ,org)
                        print("renk")
                        deger=deger+1;
        #cikti=cv2.bitwise_and(image,hsv_copy,mask=maske)
        #cv2.imshow("hsv",cikti)
        #cv2.imshow("1",hsv_copy)
        #cv2.imshow("orginal",org)

        key=cv2.waitKey(1) & 0xFF
        #print(60/((b-a)*1000))
        rawCapture.truncate(0)
        if key == ord("q"):
                break
