import rospy
import random
from bwi_tabletop_perception.srv import ShelftopPerception, ShelftopPerceptionRequest, PerceiveTabletopRequest, \
    PerceiveTabletop


class DetectedObject:
    def __init__(self, bbox):
        self.x = bbox.pose.position.x
        self.y = bbox.pose.position.y
        self.z = bbox.pose.position.z
        self.x = bbox.pose.position.x
        self.width = bbox.scale.x
        self.length = bbox.scale.y
        self.height = bbox.scale.z
        self.label = None
        self.bbox = bbox


class PerceptionManager:
    def __init__(self):
        self.perceieve_tabletop_client = rospy.ServiceProxy('perceive_tabletop', PerceiveTabletop)
        self.perceieve_tabletop_client.wait_for_service(5)

        self.shelftop_detect_client = rospy.ServiceProxy("shelftop_object_detection_service", ShelftopPerception)
        # self.shelftop_detect_client.wait_for_service(5)

    def decide_placement(self, shelf, object_in_hand):
        # How much further (beyond having the object fully into the shelf) should we place?
        EXTRA_INTO_SHELF = shelf.scale.x / 4
        # How much extra distance should we stay away from the left and right walls of the cupboard
        CUPBOARD_WALL_MARGIN = 0.3

        min_y = -shelf.scale.y / 2 + CUPBOARD_WALL_MARGIN
        max_y = shelf.scale.y / 2 - CUPBOARD_WALL_MARGIN

        min_x = -shelf.scale.x / 2
        max_x = shelf.scale.x / 2

        x = min_x + EXTRA_INTO_SHELF
        y = min_y + random.randint(0, 4) * (max_y - min_y) / 4
        return x, y

    def perceive_tabletop(self, table):
        # What's the maximum distance from the camera which we want to consider
        # during perception? A filter on the z axis of the cloud in the camera frame
        MAX_TABLE_TO_CAMERA_DISTANCE = 3.0
        # How far above (or below if negative) the origin of the table do we
        # want to consider points?
        MINIMUM_Z = -0.1
        req = PerceiveTabletopRequest()
        req.override_filter_z = True
        req.filter_z_value = MAX_TABLE_TO_CAMERA_DISTANCE
        req.table_height = MINIMUM_Z

        try:
            res = self.perceieve_tabletop_client(req)
        # A bug in the service causes it to crash instead of returning no plane
        except rospy.service.ServiceException as e:
            rospy.logerr("Tabletop segmentation threw an exception")
            rospy.logerr(e)
            return None
        num_objects = 0
        objects = {}

        if not res.is_plane_found:
            return None

        for aa_box, oriented_box in zip(res.tabletop_AA_bounding_box, res.tabletop_bounding_box):
            if not self.__should_keep_object(aa_box, oriented_box, table):
                rospy.loginfo("Ignoring object that doesn't fit on previously detected table...")
                continue

            name = "obj" + str(num_objects)
            num_objects += 1
            # Orientation should be identity, but for
            # some reason is 0 + 0i + 0j + 1k
            # quick and dirty fix
            aa_box.pose.orientation.x = 0
            aa_box.pose.orientation.y = 0
            aa_box.pose.orientation.z = 0
            aa_box.pose.orientation.w = 1
            objects[name] = DetectedObject(aa_box)
        return objects

    def __should_keep_object(self, aa_box, oriented_box, table):
        # HAX:
        # Floating objects don't really show up, but often the bottom of the object gets cut off and the object
        # is quasi floating. Reasonably detections have the objcets 15cm above the table sometimes (!).
        # Seems to be worse with higher tables
        MAX_DISTANCE_ABOVE_TABLE = .5
        table_x_limit = table.scale.x / 2
        table_y_limit = table.scale.y / 2

        x = oriented_box.pose.position.x
        y = oriented_box.pose.position.y
        z = oriented_box.pose.position.z
        # Is the center on the table? Is it a reasonable height off the table
        object_to_table_dist = z - oriented_box.scale.z / 2.0
        if abs(x) > table_x_limit or abs(y) > table_y_limit:
            rospy.loginfo("Object doesn't fit on surface")
            return False
        if object_to_table_dist > MAX_DISTANCE_ABOVE_TABLE:
            rospy.loginfo("Object is too far above the table")
            return False
        return True

    def collision_mapping(self, shelf):
        return None
        try:
            shelf_detect_response = self.shelftop_detect_client(shelf.pose.position.z)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return None
        return shelf_detect_response.cloud_plane
