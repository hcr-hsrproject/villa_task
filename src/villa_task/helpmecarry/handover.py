import rospy
from hsrb_interface import Robot, exceptions

class HandoverManager(object):
    def __init__(self,omnibase,whole_body,gripper):
        self.to_width =0.0
        self.body = whole_body
        self.omnibase = omnibase
        self.gripper = gripper
        # self.gripper.command()

    def execute_handover(self):
        self.body.move_to_joint_positions({"arm_lift_joint": 0.4, "arm_flex_joint": -0.3})
        self.open_gripper(1.0)
        rospy.sleep(15)
        self.close_gripper(0.2)

    def open_gripper(self, to_width=1.2):
        self.gripper.command(to_width)

    def close_gripper(self, to_width=0.2):
        self.gripper.command(to_width)
