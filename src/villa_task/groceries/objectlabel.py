import rospy
import numpy as np
import collections
import cv2
import tf2_geometry_msgs
import xml.etree.ElementTree as ET
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from tmc_yolo2_ros.msg import Detections
from visualization_msgs.msg import MarkerArray

from villa_task.groceries import visualization

CAMERAS = {
    'xtion': {
        'frame': 'head_rgbd_sensor_rgb_frame',
        'image_topic': '/hsrb/head_rgbd_sensor/rgb/image_rect_color',
        'projection_map': np.array(
            [[537.4419394922311, 0.0, 320.7446731239431, 0.0], [0.0, 537.3267489282307, 236.6190392583983, 0.0],
             [0.0, 0.0, 1.0, 0.0]])
    },
    'lstereo': {
        'frame': 'head_l_stereo_camera_frame',
        'image_topic': '/hsrb/head_l_stereo_camera/image_rect_color',
        'projection_map': np.array(
            [[975.7003719807848, 0.0, 640.0, 0.0], [0.0, 975.7003719807848, 480.0, 0.0], [0.0, 0.0, 1.0, 0.0]])}
}


"""
Deals with table object labelling, shelf categorization, and image annotation
:param categories_file - xml file of categories/objects
"""
class ObjectLabeller(object):
    def __init__(self, categories_file, camera='lstereo', debug=False):
        self.camera = camera
        self.categories_file = categories_file

        self.debug = debug
        self.yolo_detections = []
        # Create yolo detections subscriber
        rospy.Subscriber("/yolo2_node/detections", Detections, self.__yolo_detections_callback)

        # Subscribe to camera image
        rospy.Subscriber(CAMERAS[self.camera]['image_topic'], Image, self.__camera_image_callback)

        # Subscribe to yolo images for annotation purposes
        self.bridge = CvBridge()
        if self.debug:
            rospy.Subscriber("/yolo2_image_node/image_result", Image, self.__yolo_image_callback)


    def annotate_image(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, self.camera_image.encoding)

        for d in self.yolo_detections:
            category = self.__find_object_category(d.class_name)

            # TODO: Category should never be None as YOLO should return valid objects
            if category is None:
                continue

            cv2.rectangle(cv_image,
                    (int(d.x - d.width / 2), int(d.y - d.height / 2)),
                    (int(d.x + d.width / 2), int(d.y + d.height / 2)),
                    (255, 255, 255), thickness=2)
            cv2.putText(cv_image,
                    category,
                    (int(d.x), int(d.y)),
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.6,
                    (255, 255, 255))

        if self.debug:
            cv2.imshow('Image', cv_image)
            cv2.waitKey()
        return self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")


    def project_shelves(self, shelves, transform_manager):
        cam_to_map = transform_manager.get_transform(CAMERAS[self.camera]['frame'], 'map')
        if self.debug:
            cv_image = self.bridge.imgmsg_to_cv2(self.yolo_image, self.yolo_image.encoding)
        sorted_shelves = sorted(shelves, key=lambda s: s.name)

        shelf_positions = []
        missedup = []
        misseddown = []

        for idx, shelf in enumerate(sorted_shelves):
            shelf_pos = Point(shelf.bbox.pose.position.x - shelf.bbox.scale.x/2, shelf.bbox.pose.position.y, shelf.bbox.pose.position.z)
            ps = PointStamped()
            ps.header.frame_id = '/map'
            ps.point = shelf_pos
            cam_point = tf2_geometry_msgs.do_transform_point(ps, cam_to_map)

            center = self.__project_3d_point(cam_point.point)
            out_of_bounds = True
            if center.y > self.camera_image.height:
                misseddown.append((idx, shelf))
            elif center.y < 0:
                missedup.append((idx, shelf))
            else:
                out_of_bounds = False

            shelf_positions.append((shelf, center, out_of_bounds))

            if self.debug:
                cv2.circle(cv_image, (int(center.x), int(center.y)), 5, 255, thickness=2)

        # Classify based on detections
        categories = [[] for _ in shelf_positions]
        print(len(self.yolo_detections))
        for d in self.yolo_detections:
            category = self.__find_object_category(d.class_name)

            if category is None:
                continue

            # Pick the closest shelf below the object
            closest = -1
            mindist = float('inf')
            for idx, sp in enumerate(shelf_positions):
                _, pos, out_of_bounds = sp

                if out_of_bounds:
                    continue

                if d.y < pos.y and pos.y - d.y < mindist:
                    closest = idx
                    mindist = pos.y - d.y

            if closest >= 0:
                categories[closest].append(category)

        for idx, c in enumerate(categories):
            try:
                category = max(c, key=c.count)
            except ValueError:
                category = ''

            sorted_shelves[idx].category = category

        if self.debug:
            cv2.imshow('Image', cv_image)
            cv2.waitKey()

        return (missedup, misseddown)

    def label_objects(self, transform_manager, objects):
        labeled_objects = {}

        print(CAMERAS[self.camera]['frame'])
        cam_to_table = transform_manager.get_transform(CAMERAS[self.camera]['frame'], 'table')

        if self.debug:
            cv_image = self.bridge.imgmsg_to_cv2(self.yolo_image, self.yolo_image.encoding)

        for name, obj in objects.iteritems():
            # Construct point for tf2 transform
            obj_pos = Point(obj.x, obj.y, obj.z)
            ps = PointStamped()
            ps.header.frame_id = '/table'
            ps.point = obj_pos
            cam_point = tf2_geometry_msgs.do_transform_point(ps, cam_to_table)

            # Transform 3d bounding box to camera frame
            center = self.__transform_table_point(obj.x, obj.y, obj.z, cam_to_table)
            left_corner = self.__transform_table_point(obj.x - obj.width / 2, obj.y - obj.length / 2, obj.z - obj.height / 2, cam_to_table)
            right_corner = self.__transform_table_point(obj.x + obj.width / 2, obj.y + obj.length / 2, obj.z + obj.height / 2, cam_to_table)

            # Project bounding box
            img_center = self.__project_3d_point(center)
            img_left_corner = self.__project_3d_point(left_corner)
            img_right_corner = self.__project_3d_point(right_corner)

            if self.debug:
                cv2.circle(cv_image, (int(img_center.x), int(img_center.y)), 5, 255, thickness=2)
                cv2.rectangle(cv_image,
                        (int(img_left_corner.x), int(img_left_corner.y)),
                        (int(img_right_corner.x), int(img_right_corner.y)),
                        255, thickness=2)

            # Associate labels
            label = ''
            mindist = float('inf')  # Min distance from center of yolo bounding box
            for d in self.yolo_detections:
                if self.__within_bounding_box((img_center.x, img_center.y), (d.x, d.y), (d.width, d.height)):

                    # If the enclosing bounding box is much larger than the projected 3d box, then ignore (e.g. entire table box)
                    yoloarea = d.width*d.height
                    objarea = abs(img_right_corner.x - img_left_corner.x) * abs(img_right_corner.y - img_left_corner.y)
                    if yoloarea > 10*objarea:
                        rospy.loginfo("objectlable: Filtering out too large enclosing bounding box %f vs %f" % (yoloarea, objarea))
                        continue

                    dist = (img_center.x - d.x) * (img_center.x - d.x) + (img_center.y - d.y) * (img_center.y - d.y)
                    # We want the tightest bounding box
                    if dist < mindist:
                        mindist = dist
                        label = d.class_name

            labeled_objects[name] = label


        #if self.debug:
        #    cv2.imshow('Image', cv_image)
        #    cv2.waitKey()

        markerarray = MarkerArray()  # MarkerArray for RVIZ
        for name, label in labeled_objects.iteritems():
            markerarray.markers.append(visualization.construct_rviz_marker(label, int(name[-1]), objects[name].x, objects[name].y, objects[name].z))
        visualization.object_labels_pub.publish(markerarray)
        return labeled_objects


    # Finds the category for an object
    def __find_object_category(self, object_name):
        #parse xml
        try:
            self.category_tree
        except AttributeError:
            self.category_tree = ET.parse(self.categories_file)

        #get root (rooms in this case)
        root = self.category_tree.getroot()

        for cat in root.findall("./category"):
            for obj in cat:
                if obj.attrib['name'] == object_name:
                    return cat.attrib['name']
        return None


    def __project_3d_point(self, point):
        # Range subspace of the projection map is the camera image plane, so we can use this to project
        # Note: 3d point must be in camera frame
        u, v, w = np.dot(CAMERAS[self.camera]['projection_map'], [point.x, point.y, point.z, 1.])
        img_x = u / w
        img_y = v / w
        return Point(img_x, img_y, 0.)

    def __transform_table_point(self, x, y, z, transform):
            obj_pos = Point(x, y, z)
            ps = PointStamped()
            ps.header.frame_id = '/table'
            ps.point = obj_pos
            cam_point = tf2_geometry_msgs.do_transform_point(ps, transform)
            return cam_point.point

    def __within_bounding_box(self, test_point, center, size):
        x, y = test_point
        c_x, c_y = center
        width, height = size
        return c_x - width / 2 <= x and x <= c_x + width / 2 and c_y - height / 2 <= y and y <= c_y + height / 2

    def __camera_image_callback(self, data):
        self.camera_image = data

    def __yolo_detections_callback(self, data):
        self.yolo_detections = data.detections

    def __yolo_image_callback(self, data):
        self.yolo_image = data
