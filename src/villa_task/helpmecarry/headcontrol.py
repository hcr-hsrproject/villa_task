#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty

class HeadControl():
    def __init__(self):
        self.head_on = True
    
    def stop(self):
        rospy.wait_for_service('/viewpoint_controller/stop')
        stop_head = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        stop_head()
        
    def start(self):
        rospy.wait_for_service('/viewpoint_controller/start')
        start_head = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
        start_head()