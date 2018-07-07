#!/usr/bin/env python
import rospy
import pygame
import Adafruit_I2C
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, StopLineReading
from std_msgs.msg import String, Int32, Int16
from sensor_msgs.msg import Joy,CompressedImage
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
from duckietown_utils.jpg import image_cv_from_jpg
import time
import urllib2
import cv2
import numpy as np
import yaml
import math
from __builtin__ import True

class bot_arm_camera(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.sub_image=rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.ArmImage, queue_size=1)
        self.sub_joy=rospy.Subscriber("joy",Joy, self.camera_action, queue_size=1)
        self.pub_joy_cmd = rospy.Publisher("joy_mapper_node/car_cmd",Twist2DStamped, queue_size=1)
        self.r=0
        self.l=0
        self.m=0
        self.action=True
        self.camera_time=time.time()
        #self.yamltest=rospy.get_param("~yamltest")
    def camera_action(self,joy_msg):
        if(joy_msg.buttons[0]==1):
            print("111111111111111111111111111111111111111111111111")
            self.action=~self.action
            print(self.action)
    def ArmImage(self,image_msg):
        if self.action==1:
            return
        if time.time()-self.camera_time<0.5:
            return       
        self.camera_time=time.time()
        image_cv=image_cv_from_jpg(image_msg.data)
        img_data=np.copy(image_cv)
        hsv=np.empty(0)
        hsv=cv2.cvtColor(img_data,cv2.COLOR_BGR2HSV)
        low_blue=np.array([90,80,80])
        high_blue=np.array([130,255,255])
        mask=cv2.inRange(hsv,low_blue,high_blue)
        img,contours=cv2.findContours(mask,1,2)
        large_blue_area=0
        for i in range(0,len(img)):
            if cv2.contourArea(img[i])>large_blue_area:
                large_blue_area=cv2.contourArea(img[i])
                large_i=i
        if large_blue_area>300:
            moments=cv2.moments(img[large_i])
            if moments['m00']==0:
                return
            cx=moments['m10']/moments['m00']
            cy=moments['m01']/moments['m00']
            rad=(cx-320)**2+(cy-500)**2
            #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            #print(rad)
            cx-=320
            cy=700-cy
            #print(cx)
            #print(cy)
            t=cx/cy
            #print(math.atan(t))
            if cx>0:
                self.r+=1
                self.l=0
            else:
                self.r=0
                self.l+=1
            if t<=0.08 and t>=-0.08:
                self.m+=1
            else:
                self.m=0
            
            if self.r>3 and t>0.08:
                self.r=0
                cmd_msg=Twist2DStamped()
                cmd_msg.v=0.5
                cmd_msg.omega=-15
                wheel_time=time.time()
                while((time.time()-wheel_time)<0.15*t):
                    self.pub_joy_cmd.publish(cmd_msg)
                #time.sleep(0.3*t)
                cmd_msg.v=0
                cmd_msg.omega=0
                self.pub_joy_cmd.publish(cmd_msg)
                
            elif self.l>3 and t<-0.08:
                self.l=0
                cmd_msg=Twist2DStamped()
                cmd_msg.v=0.5
                cmd_msg.omega=15
                wheel_time=time.time()
                while((time.time()-wheel_time)<-0.15*t): 
                    self.pub_joy_cmd.publish(cmd_msg)
                #time.sleep(-0.3*t)
                cmd_msg.v=0
                cmd_msg.omega=0
                self.pub_joy_cmd.publish(cmd_msg)
            elif self.m>3:
                if rad>1500:
                    dis=(rad-1500)**0.5
                    print(dis)
                    cmd_msg=Twist2DStamped()
                    cmd_msg.v=0.5
                    cmd_msg.omega=0
                    wheel_time=time.time()
                    while((time.time()-wheel_time)<0.003*dis):
                        self.pub_joy_cmd.publish(cmd_msg)
                    cmd_msg.v=0
                    cmd_msg.omega=0
                    self.pub_joy_cmd.publish(cmd_msg)
                    self.m=0
        else:
            self.r=0
            self.l=0
            self.m=0



if __name__=="__main__":
    rospy.init_node("bot_arm_camera_node",anonymous=False)
    bot = bot_arm_camera()
    rospy.spin()

