#!/usr/bin/env python
import rospy
from villa_navi_service.srv import GoTargetPos_rel
from std_msgs.msg import *

class gpsrnaviManager(object):
	def __init__(self):
		self.nav_srv_client = rospy.ServiceProxy("navi_go_base", GoTargetPos_rel)
		self.nav_srv_client.wait_for_service(timeout=5)
		rospy.Subscriber('gpsr_navi_place', Int8, self.gpsr_navicallback)
		self.goal_x=self.goal_y=self.goal_t=0.5
		# self.follow_srv_cmd = std_msgs.msg.Bool()
	
	def gpsr_navicallback(self,msg):
		print("start navigation to")
		print msg.data
		self.nav_srv_client(self.goal_x,self.goal_y,self.goal_t)
		self.start_navigation(msg.data);

	def listener(self):
		rospy.spin()

	def start_navigation(self,goal):
		print goal
		if goal == 1:
		   self.goal_x = 5
		   self.goal_y = -2
		   self.goal_z = 5
		elif goal == 2:
		   self.goal_x = 0
		   self.goal_y = 0
		   self.goal_z = 0
		elif goal == 3:
		   self.goal_x = 0
		   self.goal_y = 0
		   self.goal_z = 0
		elif goal == 4:
		   self.goal_x = 0
		   self.goal_y = 0
		   self.goal_z = 0
		
		self.nav_srv_client(self.goal_x,self.goal_y,self.goal_t)


if __name__ == '__main__':
	# print("Pre Initialize node")
	rospy.init_node('gpsr_navi_service_client')
	print("Initialize node")
	gpsrnavi_manager = gpsrnaviManager()
	print("Object created")
	gpsrnavi_manager.listener()
