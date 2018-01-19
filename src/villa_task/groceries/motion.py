import rospy
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import math
import geometry_msgs
from hsrb_interface import Robot, geometry
from hsrb_interface.exceptions import MotionPlanningError
from std_srvs.srv import Empty
from villa_helpers.groovy_motion import GroovyMotion, JointLimitException
import visualization

class MotionManager:
    BASE_RADIUS = 0.21
    HEAD_MIN_PAN = -3.84
    HEAD_MAX_PAN = 1.75

    def __init__(self, omnibase, whole_body, fastmove, gripper, wrench):
        self.body = whole_body
        self.omnibase = omnibase
        self.fm = fastmove
        self.gripper = gripper
        self._force_torque_sensor = wrench
        self._stop_head_service = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        self._start_head_service = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
        self._stop_head_service.wait_for_service()
        self._start_head_service.wait_for_service()
        self.groovy = GroovyMotion()
        # We turn off the head service for the duration of the task and assume full
        # control of the head.
        self._stop_head_service()
        self._align_to_table_axis = None
        self._table_approach_pose = None
        self._align_to_shelf_axis = None
        self._shelf_approach_pose = None

    def detect_start_signal(self):
        while not rospy.is_shutdown():
            reading = self._force_torque_sensor.wrench
            if reading[0][0] > 5.0:
                break

    def decide_cupboard_approach_point(self, shelf, shelf_name):
        DISTANCE_FROM_SHELF = 0.4 + MotionManager.BASE_RADIUS
        half_cupboard_x = shelf.scale.x / 2

        target_x = - (half_cupboard_x + DISTANCE_FROM_SHELF)
        target_y = 0

        self._shelf_approach_pose = geometry.pose(x=target_x, y=target_y, z=-shelf.pose.position.z)
        visualization.publish_goal_pose(self._shelf_approach_pose, shelf_name)

    def decide_table_approach_point(self, table):
        DIST_FROM_EDGE = 0.15 + MotionManager.BASE_RADIUS
        half_table_x = table.scale.x / 2

        target_x = - (half_table_x + DIST_FROM_EDGE)
        target_y = 0

        self._table_approach_pose = geometry.pose(x=target_x, y=target_y, z=-table.pose.position.z)
        self._table_perception_pose = geometry.pose(x=target_x, y=target_y, z=-table.pose.position.z, ek=math.pi / 2.0)
        visualization.publish_goal_pose(self._table_approach_pose, 'table')

    def approach_table(self):
        try:
            self.fm.move(self._table_approach_pose, 10, ref_frame_id='table', angular_thresh=2)
        # TODO: !!!
        # FIXME: Catch the appropriate exception here
        except IOError as err:
            print "Exception occurred while moving to table"
            pass
        except Exception as err:
            print "Exception occurred while moving to table", err
            self.fm.move(self._table_approach_pose, 30, ref_frame_id='table', angular_thresh=2)
            pass

    def move_to_neutral_without_fail(self):
        try:
            self.body.move_to_neutral()
        except MotionPlanningError as e:
            rospy.logwarn("Failed to safely move to neutral. Trying unsafely")
            self.groovy.unsafe_move_to_neutral()


    def approach_table_for_perception(self, table, transform_manager):
        try:
            self.fm.move(self._table_perception_pose, 20, ref_frame_id='table', angular_thresh=1)
        # FIXME: Catch the appropriate exception here
        except IOError as err:
            "Exception occurred while moving to table"
            pass
        # look at table
        table_to_base = transform_manager.get_transform('base_link', 'table')
        look_future = self.look_at(table_to_base, table.pose, True)
        look_future(10)

    def approach_cupboard(self):
        self.fm.move(self._shelf_approach_pose, ref_frame_id="shelf0")

    # Looks at the shelves
    def observe_shelves(self, shelves, objectlabeller, transform_manager):
        # For now just look at the first shelf
        transform = transform_manager.get_transform('base_link', 'shelf0')
        look_future = self.look_at(transform, shelves[0].bbox.pose)
        look_future(10)

        rospy.sleep(4) # Time for camera to stabilize
        missedup, misseddown = objectlabeller.project_shelves(shelves, transform_manager)
        print(missedup, misseddown)


    def head_scan_cupboard(self):
        tilt_up = 0.3
        tilt_down = -0.4
        tilt_mid = (tilt_up + tilt_down) / 2.0

        # Roughly 100 degrees centered on the back of the robot
        pan_back_left = MotionManager.HEAD_MIN_PAN
        pan_back_right = -2.094
        pan_mid = (pan_back_left + pan_back_right) / 2.0

        sweep_time = 2.5

        # Initially we want to also lift the arm
        initial_traj_point = {"head_tilt_joint": tilt_up, "arm_lift_joint": 0.35, "head_pan_joint": pan_back_left, "time": 2}

        trajectory = [
            {"head_tilt_joint": tilt_up, "head_pan_joint": pan_back_left, "time": 1},
            {"head_tilt_joint": tilt_up, "head_pan_joint": pan_mid, "time": sweep_time},
            {"head_tilt_joint": tilt_up, "head_pan_joint": pan_back_right, "time": sweep_time},
            {"head_tilt_joint": tilt_mid, "head_pan_joint": pan_back_right, "time": 1},
            {"head_tilt_joint": tilt_mid, "head_pan_joint": pan_mid, "time": sweep_time},
            {"head_tilt_joint": tilt_mid, "head_pan_joint": pan_back_left, "time": sweep_time},
            {"head_tilt_joint": tilt_down, "head_pan_joint": pan_back_left, "time": 1},
            {"head_tilt_joint": tilt_down, "head_pan_joint": pan_mid, "time": sweep_time},
            {"head_tilt_joint": tilt_down, "head_pan_joint": pan_back_right, "time": sweep_time},
            {"head_tilt_joint": tilt_mid, "head_pan_joint": pan_back_right, "time": 1},
            {"head_tilt_joint": tilt_mid, "head_pan_joint": pan_mid, "time": sweep_time},
            {"head_tilt_joint": tilt_mid, "head_pan_joint": pan_back_left, "time": sweep_time},
        ]

        return self.groovy.execute_trajectory([initial_traj_point] + trajectory * 5)

    def head_scan_table(self):
        max_pan = MotionManager.HEAD_MIN_PAN + math.pi / 2.0
        min_pan = MotionManager.HEAD_MIN_PAN
        mid_pan = (min_pan + max_pan) / 2.0


        initial_traj_point = {"arm_lift_joint": 0.35, "arm_flex_joint": -0.5, "head_tilt_joint": -0.65, "head_pan_joint": max_pan, "time": 2}
        trajectory = [
            {"head_pan_joint": mid_pan, "time": 3},
            {"head_pan_joint": min_pan, "time": 3},
            {"head_pan_joint": mid_pan, "time": 3}
        ]

        return self.groovy.execute_trajectory([initial_traj_point] + trajectory * 10)

    def head_to_neutral(self):
        self.body.move_to_joint_positions({"head_tilt_joint": 0.0, "head_pan_joint": 0.0})

    def is_holding_something(self):
        GRIPPER_CLOSED_ANGLE = -0.7
        if rospy.get_param("~simulation"):
            GRIPPER_CLOSED_ANGLE = -0.05
        return self.body.joint_positions['hand_motor_joint'] > GRIPPER_CLOSED_ANGLE

    def open_gripper(self, to_width=1.2):
        self.gripper.command(to_width)

    def look_at(self, transform, object_pose, from_above=False):
        pan_angle = math.atan2(transform.transform.translation.y, transform.transform.translation.x)
        # If we need to turn more than the joint limit in the positive
        # direction, we need to go in the negative direction
        if pan_angle > MotionManager.HEAD_MAX_PAN:
            pan_angle -= 2 * math.pi
        print "Turning ", pan_angle, " to look"
        positions = {"head_pan_joint": pan_angle}
        if from_above:
            positions["arm_lift_joint"] = object_pose.position.z / 2 + 0.2
            positions["head_tilt_joint"] = -0.65 + object_pose.position.z / 5
        future = self.groovy.move_to_joint_positions(positions)

        def safe_groovy_future(timeout):
            try:
                future(timeout)
            except JointLimitException:
                rospy.logerr('Joint limits exceeded! Had to clip!')
                pass

        return safe_groovy_future
