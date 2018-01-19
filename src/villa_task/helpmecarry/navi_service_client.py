#!/usr/bin/env python
import rospy
from villa_navi_service.srv import GoTargetPos_rel
from std_msgs.msg import *

class navisrvManager(object):
	def __init__(self):
		self.nav_srv_client = rospy.ServiceProxy("navi_go_base", GoTargetPos_rel)
		self.nav_srv_client.wait_for_service(timeout=5)
		rospy.Subscriber('nav_act_cmd', Bool, self.activation_callback)
		# self.follow_srv_cmd = std_msgs.msg.Bool()
	
	def activation_callback(self,msg):
		print("start following")
		self.start_following();

	def listener(self):
		rospy.spin()

	def start_following(self):
		self.nav_srv_client(2.0, 0.2, 0.3)

if __name__ == '__main__':
	# print("Pre Initialize node")
	rospy.init_node('villa_navi_service_client')
	print("Initialize node")
	navisrv_manager = navisrvManager()
	print("Object created")
	#handover_manager.execute_handover()
	navisrv_manager.listener()
