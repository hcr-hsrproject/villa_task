import rospy
from human_filter.srv import set_target_to_follow


class HumanFollowingManager(object):
    def __init__(self):
        self.person_following_client = rospy.ServiceProxy("following_human", set_target_to_follow)
        self.person_following_client.wait_for_service(timeout=5)

    def start_following(self):
        self.person_following_client(True)
