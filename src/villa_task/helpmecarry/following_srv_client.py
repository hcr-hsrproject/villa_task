#!/usr/bin/python

import rospy
from human_filter.srv import set_target_to_follow
from std_msgs.msg import Int8
from std_msgs.msg import Bool

class HumanFollowingManager(object):
    def __init__(self):
        self.person_following_client = rospy.ServiceProxy("following_human", set_target_to_follow)
        self.person_following_client.wait_for_service(timeout=5)
        rospy.Subscriber('following_act_cmd', Bool, self.activation_callback)
        rospy.Subscriber('cmd_trackhuman', Int8, self.cmd_int_callback)
        # self.follow_srv_cmd = std_msgs.msg.Bool()
    def activation_callback(self,msg):
        print("start following")
        self.start_following();

    def cmd_int_callback(self,msg):
        print "cmd callback"
        self.person_following_client(True)

    def listener(self):
        rospy.spin()

    def start_following(self):
        self.person_following_client(True)

if __name__ == '__main__':
    # print("Pre Initialize node")
    rospy.init_node('handover_execution')
    print("Initialize node")
    following_manager = HumanFollowingManager()
    print("Object created")
    #handover_manager.execute_handover()
    following_manager.listener()
