#!/usr/bin/env python
import rospy
import pygame
import Adafruit_I2C
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, StopLineReading
from std_msgs.msg import String, Int32, Int16
from sensor_msgs.msg import Joy
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import urllib2
from __builtin__ import True

class bot_arm(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.arm_x=600
        self.arm_y=150
        self.pwm=PWM(address=0x40,debug=False)
        self.pwm.setPWMFreq(60)
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        #self.sub_joy  = rospy.Subscriber("joy", Joy, self.cdJoy, queue_size=1)
        #self.pwm=PWM(address=0x40,debug=False)
        #self.pwm.setPWM(1,0,300)
        #self.pwm.setPWMFreq(60)
        while(True):
            self.pwm.setPWM(1,0,int(self.arm_x))
            self.pwm.setPWM(2,0,int(self.arm_y))
            time.sleep(0.1)
        """
        for i in range(3) : 
            #self.pwm.setPWMFreq(60)
            self.pwm.setPWM(1,0,250)#135
            self.pwm.setPWM(2,0,450)
            self.pwm.setPWM(3,0,450)
            time.sleep(2)
            self.pwm.setPWM(1,0,600)#650
            self.pwm.setPWM(2,0,150)
            #self.pwm.setPWM(3,0,110)
            time.sleep(2) 
            print(i)
        """
        #self.pwm.setPWM(1,0,700)       
    def cbJoy(self, joy_msg):
        #print(joy_msg)
        if joy_msg.axes[3]>0.5 or joy_msg.axes[3]<-0.5 or joy_msg.axes[4]>0.5 or joy_msg.axes[4]<-0.5:
            self.arm_x-=joy_msg.axes[3]*30
            self.arm_y-=joy_msg.axes[4]*30
            if(self.arm_x>600):
                self.arm_x=600
            elif(self.arm_x<200):
                self.arm_x=200
            if(self.arm_y>600):
                self.arm_y=600
            elif(self.arm_y<200):
                self.arm_y=200
            #self.pwm.setPWM(1,0,int(self.arm_x))
            #self.pwm.setPWM(2,0,int(self.arm_y))
            #time.sleep(1)
        if(joy_msg.buttons[5]==1):
            #self.pwm.setPWM(1,0,500)
            #self.pwm.setPWM(2,0,350)
            #self.pwm.setPWM(3,0,450)
            #time.sleep(1)
            self.pwm.setPWM(3,0,100)
            #time.sleep(2)
            #self.pwm.setPWM(1,0,650)
            #self.pwm.setPWM(2,0,150)
            time.sleep(0.2)
            #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        elif(joy_msg.buttons[4]==1):
            #self.pwm.setPWM(1,0,500)
            #self.pwm.setPWM(2,0,300)
            #time.sleep(2)
            self.pwm.setPWM(3,0,450)
            #self.pwm.setPWM(1,0,650)
            #self.pwm.setPWM(2,0,150)
            time.sleep(0.2)
        
        #while True:
        #    rospy.loginfo("[%s] Initializing " %(self.node_name))
        #    print("1")

        #self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        #self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
        #self.pub_joy_override = rospy.Publisher("joy_mapper_node/joystick_override", BoolStamped, queue_size=1)
        #self.pub_voice = rospy.Publisher("~Voice", String, queue_size=1)
        #self.sub_voice = rospy.Subscriber("~Voice", String, self.cb_SoundPlayer, queue_size=1)
        #self.sub_tag_id = rospy.Subscriber("tag_detections_test", Int32, self.get_Apriltag, queue_size=1)
        #self.pub_switch = rospy.Publisher("joy_mapper_node/switchSpeed", BoolStamped, queue_size=1)
        #set global variable
        #self.pub_CommodityInfo=rospy.Publisher("commodity_info",String, queue_size=1)
        #self.sub_commodity = rospy.Subscriber("~commodity_info", String, self.salesman, queue_size=1)
        #self.pub_at_stop_line = rospy.Publisher("stop_line_filter_node/at_stop_line",BoolStamped,queue_size=1)
        #self.type_back=rospy.Publisher("open_loop_intersection_control_node/turn_type", Int16, queue_size=1)
        #self.pub_stop_line_reading = rospy.Publisher("stop_line_filter_node/stop_line_reading", StopLineReading, queue_size=1)
        #self.pub_at_stop_back = rospy.Publisher("~at_stop_back",BoolStamped,queue_size=1)
        #self.flag = 0
        #self.sound = ''
        #self.n_stop = False
        #self.path_commodity=list()
        #self.path_commodity.append(0)
        #self.car_action_check=1
        #self.back_info=0
        #self.last_commodity_tag=0
        #self.car_read_action()
        #self.last_action=""       
        #self.path_commodity=None
if __name__=="__main__":
    rospy.init_node("bot_arm_node",anonymous=False)
    bot = bot_arm()
    rospy.spin()

