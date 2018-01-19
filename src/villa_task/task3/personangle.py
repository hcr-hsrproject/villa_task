#!/usr/bin/env python

"""
Turn towards the closest person found by yolo
"""

import rospy 
from std_msgs.msg import String, Bool
import time
import math
from hsrb_interface import Robot
from tmc_yolo2_ros.msg import Detections
import numpy as np
from tmc_geometry_msgs.msg import Point2DStamped
from tmc_perspective_transformer.srv import (InversePerspectiveTransform, InversePerspectiveTransformRequest)
import sys

# Camera projection matrix from calibration
projection = np.matrix([[537.4419394922311,0.0,320.7446731239431,0.0],[0.0,537.3267489282307,236.6190392583983,0.0],[0.0,0.0,1.0,0.0]])
projection_inv = np.linalg.pinv(projection)


class LocatePerson():
    def __init__(self, fastmove):
        rospy.Subscriber("/yolo2_node/detections",Detections, self.detections_callback)
        self.found_person = False
        self.current_angle = 0
        self.current_pos = 0
        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.fastmove = fastmove
        self.inv_perspective_transform_client = rospy.ServiceProxy('/inverse_perspective_transform', InversePerspectiveTransform)

    def detections_callback(self,data):
        global projection_inv
        detected_people = []
        self.found_person = False
        closest_angle = 360
        pos_count = 0
        x = 0
        y = 0
        w = 0
        h = 0
        try:
            self.inv_perspective_transform_client.wait_for_service(timeout=10)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        for detected_object in data.detections:
            if detected_object.class_name == 'person':
                self.found_person = True
                inv_perspective_transform_req = InversePerspectiveTransformRequest()
                target_point = Point2DStamped()
                target_point.point.x = detected_object.x
                target_point.point.y = detected_object.y
                inv_perspective_transform_req.points_2D.append(target_point)
                inv_perspective_transform_req.depth_registration = True
                inv_perspective_transform_req.target_frame = 'base_footprint'
                try:
                    res = self.inv_perspective_transform_client(inv_perspective_transform_req)
                except rospy.ServiceException as e:
                    rospy.logerr(e)
                    exit(1)
                point = res.points_3D[0].point

                #two_d = np.matrix([[detected_object.x],[detected_object.y],[1.0]])
                #three_d = np.dot(projection_inv,two_d)
                #angle = -math.atan(three_d[0])
                if point.x > 0 and point.x < 3:
                    angle = math.atan2(point.y, point.x)
                elif point.x >= 3 or (point.x == 0 and point.y == 0):
                    angle = 360
                else:
                    angle = math.atan2(point.y, 1.0)
                if(angle < closest_angle):
                    closest_angle = angle
                    self.current_pos = pos_count
                    x = detected_object.x
                    y = detected_object.y
                    w = detected_object.width
                    h = detected_object.height
                pos_count = pos_count + 1
        #self.current_angle = closest_angle
        if closest_angle != 360:
            self.current_angle = closest_angle
        else:
            self.current_angle = 0

        self.x = x
        self.y = y
        self.w = w
        self.h = h
        
    def turn_to_person(self):
        if(self.found_person):
            print("I found a person at angle:",self.current_angle)
            self.fastmove.go(0,0,self.current_angle, 100, relative=True)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('person_angle_node')
    turn = Turn_to_person()
    rospy.spin()
