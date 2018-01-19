import geometry_msgs.msg
import rospy
from villa_surface_detectors.srv import DetectCupboard, DetectTable
from std_msgs.msg import Float32
import math

class Detection3DManager:

    def __init__(self):
        self.table_detect_client = rospy.ServiceProxy("detect_table", DetectTable)
        self.cupboard_detect_client = rospy.ServiceProxy("detect_cupboard", DetectCupboard)
        self.table_detect_client.wait_for_service(5)
        self.cupboard_detect_client.wait_for_service(5)

    def detect_table(self, octomap_cloud, world_model):
        x, y, theta = world_model.start_pose
        ignore_origin = geometry_msgs.msg.Point(x, y, 0)
        # Rules say we start facing the cupboard, so we'll ignore a range
        # centered on theta.
        SLICE_BISECTOR = theta
        TOTAL_ANGLE_TO_IGNORE = math.pi / 2.0

        ignore_start_angle = SLICE_BISECTOR - (TOTAL_ANGLE_TO_IGNORE / 2.0)
        ignore_end_angle = SLICE_BISECTOR + (TOTAL_ANGLE_TO_IGNORE / 2.0)
        ignore_start_angle = Float32(ignore_start_angle)
        ignore_end_angle = Float32(ignore_end_angle)
        try:
            table_detect_response = self.table_detect_client(octomap_cloud, ignore_origin, ignore_start_angle, ignore_end_angle)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return None, None
        table_box = table_detect_response.table_bounding_box

        par_frame = 'map'
        child_frame = 'table'
        pose = table_box.pose
        return table_box, (par_frame, child_frame, pose)

    def detect_cupboard(self, octomap_cloud, world_model):
        x, y, theta = world_model.start_pose

        # Rules say we start facing the cupboard, so we'll ignore the angle slice of the cloud
        # centered on the opposite of the robot's starting orientation
        SLICE_BISECTOR = math.pi + theta
        TOTAL_ANGLE_TO_IGNORE = math.pi / 2.0

        ignore_origin = geometry_msgs.msg.Point(x, y, 0)
        ignore_start_angle = SLICE_BISECTOR - (TOTAL_ANGLE_TO_IGNORE / 2.0)
        ignore_end_angle = SLICE_BISECTOR + (TOTAL_ANGLE_TO_IGNORE / 2.0)
        ignore_start_angle = Float32(ignore_start_angle)
        ignore_end_angle = Float32(ignore_end_angle)
        try:
            shelf_detect_response = self.cupboard_detect_client(octomap_cloud, ignore_origin, ignore_start_angle, ignore_end_angle)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return None, None

        sorted_shelf_bboxes = sorted(shelf_detect_response.shelf_bounding_boxes, key=lambda s: s.pose.position.z)

        idx = 0
        shelves = []
        relations = []
        for shelf in sorted_shelf_bboxes:
            # Breaking works since shelves are sorted by height
            if shelf.pose.position.z > 1.7:
                break
            shelves.append(shelf)
            shelf_relation = 'map', 'shelf' + str(idx), shelf.pose
            relations.append(shelf_relation)
            idx += 1

        return shelves, relations
