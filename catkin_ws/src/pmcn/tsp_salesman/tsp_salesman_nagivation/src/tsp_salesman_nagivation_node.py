#!/usr/bin/env python
import rospy
import socket
import pygame
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from std_msgs.msg import String, Int32
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import urllib2
from __builtin__ import True

class Qwer_Player(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
        self.pub_joy_override = rospy.Publisher("joy_mapper_node/joystick_override", BoolStamped, queue_size=1)
        self.pub_voice = rospy.Publisher("~Voice", String, queue_size=1)
        self.sub_voice = rospy.Subscriber("~Voice", String, self.cb_SoundPlayer, queue_size=1)
        self.sub_tag_id = rospy.Subscriber("tag_detections_test", Int32, self.get_Apriltag, queue_size=1)
        self.pub_switch = rospy.Publisher("joy_mapper_node/switchSpeed", BoolStamped, queue_size=1)
        #set global variable
        #self.pub_CommodityInfo=rospy.Publisher("commodity_info",String, queue_size=1)
        self.sub_commodity = rospy.Subscriber("~commodity_info", String, self.salesman, queue_size=1)
        self.flag = 0
        self.sound = ''
        self.n_stop = False
        self.path_commodity=list()
        self.path_commodity.append(0)
        self.car_action_check=1
        self.car_read_action()
        
        #self.path_commodity=None
    def car_read_action(self):
        while True:
            strhttp='http://192.168.0.100/tsp/read_car_action.php?car_id=1'
            req = urllib2.Request(strhttp)
            response = urllib2.urlopen(req)
            the_page = response.read()
            #self.car_action_check=int(the_page)
            #if(self.car_action_check!=int(the_page)):
            if(True):
                self.car_action_check=int(the_page)
                e_stop_msg=BoolStamped()
                e_stop_msg.data=int(the_page)
                #self.pub_e_stop.publish(e_stop_msg)pub_lanefollowing
                self.pub_lanefollowing.publish(e_stop_msg)
                #print("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEee")
    def salesman(self,Commodity):
        #print("1111111111111111111111QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
        #print(Commodity)
        #print(Commodity.data)
        #print(Commodity.data[0])
        #print(Commodity.data[1])
        if(len(Commodity.data)):
            self.path_commodity=Commodity.data.split(" ")
        #print(self.path_commodity[0])
        #print(self.path_commodity[1])
    def get_Apriltag(self,Tag):
        i=0
        while(self.path_commodity[i]):
            #print self.path_commodity[i]
            i+=1
        tag_id = Int32()
        tag_id.data = Tag.data
        commodity_exist=0
        #print("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww")
        rospy.loginfo("Now found Id = %d-------------   and flag = %d" %(tag_id.data, self.flag))
        i=0
        tag_node=int(tag_id.data/4)
        while(len(self.path_commodity)>i+1):
            #print(len(self.path_commodity[i]))
            #print(self.path_commodity[i])
            sss=self.path_commodity[i]
            #print("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww")
            #print(self.path_commodity[i])
            #print("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww")
            if(int(sss)==tag_node):
                commodity_exist=1
                self.path_commodity[i]=-1
                break
            i=i+1
        print(commodity_exist)
        print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
        if  commodity_exist==1:
            strhttp='http://192.168.0.100/tsp/car_record_action.php?car_id=1&car_action=1'
            req = urllib2.Request(strhttp)
            response = urllib2.urlopen(req)
            the_page = response.read()
            
            strhttp='http://192.168.0.100/tsp/car_recode_navigation.php?car_id=1&tag_id='+str(tag_id.data)
            #strhttp='http://192.168.0.100/tsp/car_recode_navigation.php?car_id=1&tag_id='+str(taginfo.id)
            req = urllib2.Request(strhttp)
            response = urllib2.urlopen(req)
            the_page = response.read()
            
            self.sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/1234.mp3"
            rospy.loginfo(' ---------Found Tag playing guide vocie-----------')
            #e_stop_msg = BoolStamped()
            #e_stop_msg.data = True 
            #self.pub_e_stop.publish(e_stop_msg)
            msg = String()
            msg.data = 'at def get_apriltag(self,Tag) and tag_id.data = 350'
            self.pub_voice.publish(msg)
        if tag_id.data == 0:
            self.flag = 0
        else:
            self.flag = 1

    def cb_SoundPlayer(self,nowstr):
        pygame.mixer.init()
        rospy.loginfo("Welcome to SoundPlayer ^_^") 
        pygame.mixer.music.load(self.sound)
        pygame.mixer.music.play(0,1)
        pygame.mixer.music.fadeout(5000)
        pygame.mixer.music.set_volume(1)  #The value argument is between 0.0 and 1.0
        #pygame.mixer.music.set_endevent(self.set_event_fun)
        time.sleep(5)
        print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        # while pygame.mixer.music.get_busy():  #it will play sound until the sound finished
        # pygame.time.Clock().tick(10)           

if __name__ == "__main__":
    rospy.init_node("tsp_salesman_nagivation_node",anonymous=False)
    qwer_player = Qwer_Player()
    rospy.spin()

