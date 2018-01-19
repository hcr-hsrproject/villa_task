from villa_task.task5.speak import robot_say
import rospy
import random
from villa_surface_detectors.srv import HorizontalPlanesDetect, HorizontalPlanesDetectRequest, ShelfDetect, ShelfDetectRequest, TableDetect, TableDetectRequest


class Detection3DManager:

    def __init__(self):
        self.table_detect_client = rospy.ServiceProxy("table_detector_service", TableDetect)
        self.cupboard_detect_client = rospy.ServiceProxy("cupboard_detector_service", ShelfDetect)
        self.table_detect_client.wait_for_service(5)
        self.cupboard_detect_client.wait_for_service(5)

    def detect_table(self, octomap_cloud):
        try:
            table_detect_response = self.table_detect_client(octomap_cloud)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return None, None
        table_box = table_detect_response.table_bounding_box

        par_frame = 'map'
        child_frame = 'table'
        pose = table_box.pose
        return table_box, (par_frame, child_frame, pose)

    def detect_shelf(self, octomap_cloud):

        shelves = []
        relations = []
        try:
            shelf_detect_response = self.cupboard_detect_client(octomap_cloud)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return None, None
        idx = 0
        for shelf in shelf_detect_response.shelf_bounding_boxes:
            if shelf.pose.position.z > 1.7:
                break
            shelves.append(shelf)
            shelf_relation = 'map', 'shelf' + str(idx), shelf.pose
            relations.append(shelf_relation)
            idx += 1

        return shelves, relations
