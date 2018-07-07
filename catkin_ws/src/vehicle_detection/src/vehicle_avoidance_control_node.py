#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, BoolStamped

import os
import rospkg
import rospy
import yaml

class VehicleAvoidanceControlNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospack = rospkg.RosPack()
		self.car_cmd_pub = rospy.Publisher("~car_cmd",
				Twist2DStamped, queue_size = 1)
		self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",
				BoolStamped, queue_size=1)
		self.subscriber = rospy.Subscriber("~detection",
				BoolStamped, self.callback,  queue_size = 1)

	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def callback(self, data):
<<<<<<< HEAD
		vehicle_too_close = BoolStamped()
		vehicle_too_close.header.stamp = data.header.stamp
=======
                vehicle_too_close = BoolStamped()
                vehicle_too_close.header.stamp = data.header.stamp
>>>>>>> 2d356ff5df7440b437fd8915a698c3f4ba9b0c0c
		if not data.data:
			vehicle_too_close.data = False
		else:
			vehicle_too_close.data = True
		self.publishCmd(data.header.stamp)
		self.vehicle_detected_pub.publish(vehicle_too_close)

	def publishCmd(self,stamp): 
		cmd_msg = Twist2DStamped()
<<<<<<< HEAD
		cmd_msg.header.stamp = stamp
		cmd_msg.v = 0.0
		cmd_msg.omega = 0.0
		#print("ooooooooooooooooooooooooooooooooo")
		#print stamp
=======
                cmd_msg.header.stamp = stamp
		cmd_msg.v = 0.0
		cmd_msg.omega = 0.0
>>>>>>> 2d356ff5df7440b437fd8915a698c3f4ba9b0c0c
		self.car_cmd_pub.publish(cmd_msg)
   
if __name__ == '__main__':
	rospy.init_node('vehicle_avoidance_control_node', anonymous=False)
	controller = VehicleAvoidanceControlNode()
	rospy.spin()

