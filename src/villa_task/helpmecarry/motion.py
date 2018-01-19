import rospy
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import math
import geometry_msgs
from hsrb_interface import Robot, geometry
from std_srvs.srv import Empty
from villa_helpers.groovy_motion import GroovyMotion
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

    def decide_shelf_approach_point(self, shelf, shelf_name, transform_manager):
        curr_pose = self.omnibase.pose

        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'map'
        ps.pose.position.x = curr_pose[0]
        ps.pose.position.y = curr_pose[1]
        ps.pose.position.z = 0
        ps.pose.orientation = shelf.pose.orientation

        shelf_transform = transform_manager.get_transform(shelf_name, 'map')
        pt = tf2_geometry_msgs.do_transform_pose(ps, shelf_transform)

        x_dist, y_dist = (pt.pose.position.x, pt.pose.position.y)

        DISTANCE_FROM_SHELF = 0.4 + MotionManager.BASE_RADIUS

        half_shelf_y = shelf.scale.y / 2.0
        half_shelf_x = shelf.scale.x / 2.0
        half_pi = math.pi / 2.0
        if abs(y_dist) > abs(x_dist):
            aligned_axis = "y"
            sign = 1 if y_dist > 0 else -1
            target_x = 0
            target_y = sign * (half_shelf_y + DISTANCE_FROM_SHELF)
            theta = -sign * half_pi
        else:
            aligned_axis = "x"
            sign = 1 if x_dist > 0 else -1
            target_x = sign * (half_shelf_x + DISTANCE_FROM_SHELF)
            target_y = 0
            theta = half_pi + sign * half_pi

        self._align_to_shelf_axis = aligned_axis
        self._shelf_approach_pose = geometry.pose(x=target_x, y=target_y, z=-shelf.pose.position.z, ek=theta)
        vis_pose = PoseStamped()
        vis_pose.pose.position.x = target_x
        vis_pose.pose.position.y = target_y
        vis_pose.pose.position.z = 0.0
        vis_pose.header.frame_id = shelf_name
        vis_pose.header.stamp = rospy.Time.now()
        visualization.pose_publisher.publish(vis_pose)

    def decide_table_approach_point(self, table, transform_manager):
        map_to_table = transform_manager.get_transform('table', 'map')

        # Get current position in the map frame
        curr_pose = self.omnibase.pose

        # Make a pose with our coordinates but the table's orientation
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'map'
        ps.pose.position.x = curr_pose[0]
        ps.pose.position.y = curr_pose[1]
        ps.pose.position.z = 0
        ps.pose.orientation = table.pose.orientation

        robot_pose_wrt_table = tf2_geometry_msgs.do_transform_pose(ps, map_to_table)

        # The origin of this frame is the center of the table
        # Magnitude of the X and Y components tells us how far the robot is from the center of the table
        x_dist, y_dist = (robot_pose_wrt_table.pose.position.x, robot_pose_wrt_table.pose.position.y)

        DIST_FROM_EDGE = 0.15 + MotionManager.BASE_RADIUS
        rospy.loginfo(table.scale)

        half_table_y = table.scale.y / 2
        half_table_x = table.scale.x / 2
        half_pi = math.pi / 2.0
        # Are we further away in the X or Y dimension
        if abs(y_dist) > abs(x_dist):
            aligned_axis = "y"
            # Are we on the positive or negative side of the table?
            sign = 1 if y_dist > 0 else -1
            target_x = 0
            target_y = sign * (half_table_y + DIST_FROM_EDGE)
            theta = -sign * half_pi

        else:
            aligned_axis = "x"
            sign = 1 if x_dist > 0 else -1
            target_x = sign * (half_table_x - DIST_FROM_EDGE)
            target_y = 0

            theta = half_pi + sign * half_pi


        self._align_to_table_axis = aligned_axis
        self._table_approach_pose = geometry.pose(x=target_x, y=target_y, z=-table.pose.position.z, ek=theta)
        vis_pose = PoseStamped()
        vis_pose.pose.position.x = target_x
        vis_pose.pose.position.y = target_y
        vis_pose.pose.position.z = 0.0
        vis_pose.header.frame_id = "table"
        vis_pose.header.stamp = rospy.Time.now()
        visualization.pose_publisher.publish(vis_pose)

    def move_to_table(self):
        try:
            self.fm.move(self._table_approach_pose, 20, ref_frame_id='table', angular_thresh=5)
        # FIXME: Catch the appropriate exception here
        except IOError as err:
            "Exception occurred while moving to table"
            pass

    def move_to_shelf(self, shelf, shelf_name):
        self.fm.move(self._shelf_approach_pose, ref_frame_id=shelf_name)

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

        # 100 times :P
        return self.groovy.execute_trajectory([initial_traj_point] + trajectory * 100)

    def head_scan_table(self):
        start_pan = MotionManager.HEAD_MIN_PAN + 3.14 / 2.0
        mid_pan = (MotionManager.HEAD_MIN_PAN + start_pan) / 2.0

        trajectory = [
            {"arm_lift_joint": 0.35, "arm_flex_joint": -0.5, "head_tilt_joint": -0.65, "head_pan_joint": start_pan, "time": 3},
            {"head_pan_joint": mid_pan, "time": 2},
            {"head_pan_joint": MotionManager.HEAD_MIN_PAN, "time": 2},
            {"head_pan_joint": MotionManager.HEAD_MIN_PAN, "time": 0.5} # Same goal as last to add delay
        ]

        return self.groovy.execute_trajectory(trajectory)

    def head_to_neutral(self):
        self.body.move_to_joint_positions({"head_tilt_joint": 0.0, "head_pan_joint": 0.0})

    def grasp_failed(self):
        return self.body.joint_positions['hand_motor_joint'] <= -0.7

    def open_gripper(self, obj_width=1.2):
        if obj_width < 0.032:
            amount = 0.3
        elif obj_width < 0.042:
            amount = 0.4
        elif obj_width < 0.062:
            amount = 0.6
        elif obj_width < 0.092:
            amount = 0.8
        elif obj_width < 0.112:
            amount = 1.0
        else:
            amount = 1.2
        self.gripper.command(amount)

    def look_at(self, transform, object_height=None):
        object_height = None
        pan_angle = math.atan2(transform.transform.translation.y, transform.transform.translation.x)
        # If we need to turn more than the joint limit in the positive
        # direction, we need to go in the negative direction
        if pan_angle > MotionManager.HEAD_MAX_PAN:
            pan_angle -= 2 * math.pi
        print "Turning ", pan_angle, " to look"
        positions = {"head_tilt_joint": -0.65,
                     "head_pan_joint": pan_angle}
        if object_height is not None:
            positions["arm_lift_joint"] = object_height + 0.2
        self.body.move_to_joint_positions(positions)
