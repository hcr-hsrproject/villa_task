import operator
import geometry_msgs
import math
import tf2_geometry_msgs
from hsrb_interface import geometry
import tf, rospy
from tf import transformations
from villa_task.groceries.motion import MotionManager

class Shelf:
    def __init__(self, name, bbox, category=''):
        self.name = name
        self.bbox = bbox
        self.category = category

class WorldModel:
    def __init__(self, collision_world, transform_manager):
        self.shelves = None
        self.shelf_contents = None
        self.table = None
        self.table_objects = {}
        self.current_object = None
        self.collision_world = collision_world
        self._collision_object_handles = {}
        self.transform_manager = transform_manager
        self.start_pose = None

    def clear_table_objects(self):
        for object_name, object in self.table_objects.items():
            if object_name in self._collision_object_handles.keys():
                collision_handle = self._collision_object_handles[object_name]
                self.collision_world.remove(collision_handle)
                del self._collision_object_handles[object_name]
            self.transform_manager.remove_relation_by_object_name(object_name)

    def add_table(self, table, relation):
        self.transform_manager.add_relation(relation)
        self.transform_manager.broadcast_links()
        table, relation = self.normalize_table(table, relation)
        self.table = table
        self.transform_manager.add_relation(relation)
        self.transform_manager.broadcast_links()
        self._add_table_collision(table)


    def add_shelves(self, shelves, shelf_relations):
        shelf_names = [name for _, name, _ in shelf_relations]
        self.shelves = [Shelf(n, s) for n, s in zip(shelf_names, shelves)]
        self.transform_manager.add_relations(shelf_relations)
        self.transform_manager.broadcast_links()

        # Normalize shelves with the base axes
        normalized_shelves = []
        for i in range(len(shelves)):
            _, shelf_name, _ = shelf_relations[i]
            shelf, relation = self.normalize_shelf(shelves[i], shelf_name, shelf_relations[i])
            self.transform_manager.add_relation(relation)
            self.transform_manager.broadcast_links()
            normalized_shelves.append(shelves[i])

        self._add_cupboard_collision(normalized_shelves)

        self.shelf_contents = [[] for _ in range(len(shelves))]

    def add_table_objects(self, objects):
        self.table_objects = objects
        for object_name, obj in objects.items():
            new_relation = self._add_table_object_collision(obj, object_name)
            self.transform_manager.add_relation(new_relation)
        self.transform_manager.broadcast_links()

    def sort_table_objects(self, axis, robot_pose):
        obj_dict = {}
        map_to_table = self.transform_manager.get_transform('table', 'map')

        # Make a pose with our coordinates but the table's orientation
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'map'
        ps.pose.position.x = robot_pose[0]
        ps.pose.position.y = robot_pose[1]
        ps.pose.position.z = 0
        ps.pose.orientation.x = self.table.pose.orientation.x
        ps.pose.orientation.y = self.table.pose.orientation.y
        ps.pose.orientation.z = self.table.pose.orientation.z
        ps.pose.orientation.w = self.table.pose.orientation.w

        robot_pose_wrt_table = tf2_geometry_msgs.do_transform_pose(ps, map_to_table)
        for obj in self.table_objects:
            obj_dict[obj] = self.table_objects[obj].width + abs(robot_pose_wrt_table.pose.position.x-self.table_objects[obj].x)
       
        sorted_objects = sorted(obj_dict.items(), key=operator.itemgetter(1))
        print sorted_objects
        return sorted_objects

    def get_sorted_target_shelves_for_object(self, object_name):
        target_shelves = []
        shelves_bboxes = [s.bbox for s in self.shelves]
        for shelf, contents, index in zip(shelves_bboxes, self.shelf_contents, list(range(0, len(self.shelves)))):
            if len(contents) == 0:
                target_shelves.append((shelf, index))
        return target_shelves

    def set_axis(self, axis):
        self.axis = axis

    def disable_collision_for(self, object_name):
        if object_name in self._collision_object_handles.keys():
            collision_handle = self._collision_object_handles[object_name]
            self.collision_world.remove(collision_handle)
            del self._collision_object_handles[object_name]

    def picked_up_object(self, object_name):
        self.current_object = self.table_objects[object_name]
        del self.table_objects[object_name]
        self.transform_manager.remove_relation_by_object_name(object_name)

    def placed_current_object(self, shelf_index):
        current_contents = self.shelf_contents[shelf_index]
        current_contents.append(self.current_object)
        self.shelf_contents[shelf_index] = current_contents
        self.current_object = None

    def _add_table_collision(self, table):
        rot = tf.transformations.euler_from_quaternion([table.pose.orientation.x,
                                                        table.pose.orientation.y,
                                                        table.pose.orientation.z,
                                                        table.pose.orientation.w])
        pose = geometry.pose(x=table.pose.position.x,
                             y=table.pose.position.y,
                             z=table.pose.position.z / 2 + 0.01,
                             ei=rot[0], ej=rot[1], ek=rot[2])
        self._collision_object_handles["table"] = self.collision_world.add_box(x=table.scale.x, y=table.scale.y,
                                                                               z=table.pose.position.z + 0.02,
                                                                               pose=pose,
                                                                               frame_id='map')

    def _add_table_object_collision(self, object, objectname):
        # Too small and the robot will collide with objects,
        # too large and he'll fail to plan (the bounding box is usually an overestimate of true bounds)
        COLLISION_OBJECT_SHRINK_FACTOR = 0.8
        rot = tf.transformations.euler_from_quaternion([object.bbox.pose.orientation.x,
                                                        object.bbox.pose.orientation.y,
                                                        object.bbox.pose.orientation.z,
                                                        object.bbox.pose.orientation.w])
        new_pose = geometry.pose(x=object.x, y=object.y, z=object.z,
                                 ei=rot[0], ej=rot[1], ek=rot[2])
        handle = self.collision_world.add_box(x=object.width * COLLISION_OBJECT_SHRINK_FACTOR, y=object.length * COLLISION_OBJECT_SHRINK_FACTOR, z=object.height,
                                              pose=new_pose, frame_id='table')
        self._collision_object_handles[objectname] = handle
        par_frame = 'table'
        child_frame = objectname
        return par_frame, child_frame, object.bbox.pose

    def _add_cupboard_collision(self, shelves):

        min_x = min_y = min_z = float("inf")
        max_x = max_y = max_z = -float("inf")

        for shelf in shelves:
            position = shelf.pose.position
            orientation = shelf.pose.orientation
            max_x = max(max_x, position.x + shelf.scale.x / 2.0)
            max_y = max(max_y, position.y + shelf.scale.y / 2.0)
            max_z = max(max_z, position.z + shelf.scale.z / 2.0)
            min_x = min(min_x, position.x - shelf.scale.x / 2.0)
            min_y = min(min_y, position.y - shelf.scale.y / 2.0)
            min_z = min(min_z, position.z - shelf.scale.z / 2.0)
            rot = tf.transformations.euler_from_quaternion([orientation.x,
                                                            orientation.y,
                                                            orientation.z,
                                                            orientation.w])
            new_pose = geometry.pose(x=position.x, y=position.y, z=position.z,
                                     ei=rot[0], ej=rot[1], ek=rot[2])
            # TODO: We aren't keeping the handles for these objects so we can't actually remove them
            self.collision_world.add_box(x=shelf.scale.x, y=shelf.scale.y, z=shelf.scale.z + 0.02,
                                         pose=new_pose, frame_id='map')

        shelf = shelves[0]
        shelf_frame = "shelf0"
        if shelf.scale.x > shelf.scale.y:
            x_scale = 0.04
            y_scale = shelf.scale.y + shelf.scale.x 
            negative_pose = geometry.pose(x=-shelf.scale.x / 2, y=0, z=0.9 - shelf.pose.position.z)
            positive_pose = geometry.pose(x=shelf.scale.x / 2, y=0, z=0.9 - shelf.pose.position.z)
        else:
            x_scale = shelf.scale.x + shelf.scale.y 
            y_scale = 0.04
            negative_pose = geometry.pose(x=0, y=-shelf.scale.y / 2, z=0.9 - shelf.pose.position.z)
            positive_pose = geometry.pose(x=0, y=shelf.scale.y / 2, z=0.9 - shelf.pose.position.z)
        self.collision_world.add_box(x=x_scale, y=y_scale, z=1.8, pose=negative_pose, frame_id=shelf_frame)
        self.collision_world.add_box(x=x_scale/2, y=y_scale, z=1.8, pose=positive_pose, frame_id=shelf_frame)


    def normalize_table(self, table, relation):
        map_to_table = self.transform_manager.get_transform('table', 'map')

        curr_pose = self.start_pose

        # Make a pose with our coordinates but the table's orientation
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'map'
        ps.pose.position.x = curr_pose[0]
        ps.pose.position.y = curr_pose[1]
        ps.pose.position.z = 0
        ps.pose.orientation.x = table.pose.orientation.x
        ps.pose.orientation.y = table.pose.orientation.y
        ps.pose.orientation.z = table.pose.orientation.z
        ps.pose.orientation.w = table.pose.orientation.w

        robot_pose_wrt_table = tf2_geometry_msgs.do_transform_pose(ps, map_to_table)

        euler = tf.transformations.euler_from_quaternion([table.pose.orientation.x,
                                                        table.pose.orientation.y,
                                                        table.pose.orientation.z,
                                                        table.pose.orientation.w])
        yaw = euler[2]

        # The origin of this frame is the center of the table
        # Magnitude of the X and Y components tells us how far the robot is from the center of the table
        x_dist, y_dist = (robot_pose_wrt_table.pose.position.x, robot_pose_wrt_table.pose.position.y)


        half_pi = math.pi / 2.0
        # Which edge of the table are we closer to
        if abs(y_dist) > abs(x_dist):
            aligned_axis = "y"
            old_x_scale = table.scale.x
            old_y_scale = table.scale.y
            table.scale.x = old_y_scale
            table.scale.y = old_x_scale
            # Are we on the positive or negative side of the table ?
            sign = 1 if y_dist > 0 else -1
            if sign is 1:
                yaw -= half_pi
            else:
                yaw += half_pi
        else:
            aligned_axis = "x"
            sign = 1 if x_dist > 0 else -1
            if sign is 1:
                yaw += math.pi
            else:
                pass

        new_orientation = transformations.quaternion_from_euler(euler[0], euler[1], yaw)
        new_pose = relation[2]
        new_pose.orientation.x = new_orientation[0]
        new_pose.orientation.y = new_orientation[1]
        new_pose.orientation.z = new_orientation[2]
        new_pose.orientation.w = new_orientation[3]
        new_relation = (relation[0], relation[1], new_pose)
        table.pose = new_pose
        return table, new_relation

    def normalize_shelf(self, shelf, shelf_name, relation):
        map_to_shelf = self.transform_manager.get_transform(shelf_name, 'map')

        curr_pose = self.start_pose

        # Make a pose with our coordinates but the shelf's orientation
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'map'
        ps.pose.position.x = curr_pose[0]
        ps.pose.position.y = curr_pose[1]
        ps.pose.position.z = 0
        ps.pose.orientation.x = shelf.pose.orientation.x
        ps.pose.orientation.y = shelf.pose.orientation.y
        ps.pose.orientation.z = shelf.pose.orientation.z
        ps.pose.orientation.w = shelf.pose.orientation.w

        robot_pose_wrt_shelf = tf2_geometry_msgs.do_transform_pose(ps, map_to_shelf)

        euler = tf.transformations.euler_from_quaternion([shelf.pose.orientation.x,
                                                        shelf.pose.orientation.y,
                                                        shelf.pose.orientation.z,
                                                        shelf.pose.orientation.w])
        yaw = euler[2]

        # The origin of this frame is the center of the shelf
        # Magnitude of the X and Y components tells us how far the robot is from the center of the shelf
        x_dist, y_dist = (robot_pose_wrt_shelf.pose.position.x, robot_pose_wrt_shelf.pose.position.y)


        half_pi = math.pi / 2.0
        # Which edge of the shelf are we closer to
        if abs(y_dist) > abs(x_dist):
            aligned_axis = "y"
            old_x_scale = shelf.scale.x
            old_y_scale = shelf.scale.y
            shelf.scale.x = old_y_scale
            shelf.scale.y = old_x_scale
            # Are we on the positive or negative side of the shelf?
            sign = 1 if y_dist > 0 else -1
            if sign is 1:
                yaw -= half_pi
            else:
                yaw += half_pi
        else:
            aligned_axis = "x"
            sign = 1 if x_dist > 0 else -1
            if sign is 1:
                yaw += math.pi
            else:
                pass

        new_orientation = transformations.quaternion_from_euler(euler[0], euler[1], yaw)
        new_pose = relation[2]
        new_pose.orientation.x = new_orientation[0]
        new_pose.orientation.y = new_orientation[1]
        new_pose.orientation.z = new_orientation[2]
        new_pose.orientation.w = new_orientation[3]
        new_relation = (relation[0], relation[1], new_pose)
        shelf.pose = new_pose
        return shelf, new_relation
