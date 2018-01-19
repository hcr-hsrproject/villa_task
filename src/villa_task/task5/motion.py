import rospy
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import math
import geometry_msgs
from hsrb_interface import Robot, geometry


class MotionManager:
    def __init__(self, omnibase, whole_body, fastmove, gripper, wrench):
        self.body = whole_body
        self.omnibase = omnibase
        self.fm = fastmove
        self.gripper = gripper
        self._force_torque_sensor = wrench


    def detect_start_signal(self):
        while not rospy.is_shutdown():
            reading = self._force_torque_sensor.wrench
            if reading[0][0] > 10.0:
                break
            pass

    def move_to_table(self, table, transform_manager):
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


        DIST_FROM_EDGE = 1.0
        theta = 0
        aligned_axis = "x"
        # Are we further away in the X or Y dimension
        if abs(y_dist) > abs(x_dist):
            print " aligning to y axis"
            aligned_axis = "y"
            if y_dist < 0:
                target_x = 0
                target_y = table.scale.y / 2 - DIST_FROM_EDGE
                theta = 1.57
            else:
                target_x = 0
                target_y = table.scale.y / 2 + DIST_FROM_EDGE
                theta = -1.57
        else:
            print " aligning to x axis"
            aligned_axis = "x"
            if x_dist < 0:
                target_x = table.scale.x / 2 - DIST_FROM_EDGE
                target_y = 0
                theta = 0
            else:
                target_x = table.scale.x / 2 + DIST_FROM_EDGE
                target_y = 0
                theta = 3.14

        print "delta:", target_x, ",", target_y, " theta: ", theta
        try:
            target_pose = geometry.pose(x=target_x, y=target_y, z=-table.pose.position.z, ek=theta)
            self.fm.move(target_pose, 8, angular_thresh=0.2, ref_frame_id='table')
        #FIXME: Catch the appropriate exception here
        except IOError as err:
            "Exception occurred while moving to table"
            pass

        return theta, aligned_axis, (x_dist, y_dist), target_x, target_y

    def move_to_shelf(self, shelf, shelf_name, transform_manager):
        curr_pose = self.omnibase.pose

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

        shelf_transform = transform_manager.get_transform(shelf_name, 'map')
        pt = tf2_geometry_msgs.do_transform_pose(ps, shelf_transform)

        dist = (pt.pose.position.x, pt.pose.position.y)

        theta = 0

        if abs(dist[1]) > abs(dist[0]):
            print " aligning to y axis"
            if dist[1] < 0:
                delta_x = 0
                delta_y = -shelf.scale.y / 2 - 0.6
                theta = 1.57
            else:
                delta_x = 0
                delta_y = shelf.scale.y / 2 + 0.6
                theta = -1.57
        else:
            print " aligning to x axis"
            if dist[0] < 0:
                delta_x = -shelf.scale.x / 2 - 0.6
                delta_y = 0
                theta = 0
            else:
                delta_x = shelf.scale.x / 2 + 0.6
                delta_y = 0
                theta = 3.14

        print "delta:", delta_x, ",", delta_y, " theta: ", theta

        self.fm.move(geometry.pose(x=delta_x, y=delta_y, z=-shelf.pose.position.z, ek=theta), ref_frame_id=shelf_name, angular_thresh=0.2)



    def look_backwards(self):
        self.body.end_effector_frame = u'hand_palm_link'
        self.body.move_to_joint_positions({"head_tilt_joint": 0.35, "arm_lift_joint": 0.35, "head_pan_joint": -3.14})
        self.body.move_to_joint_positions({"head_tilt_joint": -0.25, "head_pan_joint": -3.14})

        self.body.move_to_joint_positions({"head_tilt_joint": -0.5, "head_pan_joint": -3.14})

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

    def head_scan_table(self):
        delay = 1
        self.body.move_to_joint_positions({"arm_lift_joint":0.35, "arm_flex_joint":-0.5, "head_tilt_joint":-0.35, "head_pan_joint":-2.35})
        self.body.move_to_joint_positions({"head_tilt_joint":-0.65, "head_pan_joint":-2.35})
        rospy.sleep(delay)
        self.body.move_to_joint_positions({"head_tilt_joint":-0.65, "head_pan_joint":-3.24})
        rospy.sleep(delay)
        self.body.move_to_joint_positions({"head_tilt_joint":-0.65, "head_pan_joint":-3.64})
        rospy.sleep(delay/2)

    def look_at(self, transform, object_height=None):
        object_height = None
        pan_angle = math.atan2(transform.transform.translation.y,transform.transform.translation.x)
        # If we need to turn more than the joint limit in the positive
        # direction, we need to go in the negative direction
        if pan_angle > 1.75:
            pan_angle -= 2*3.14
        print "Turning ", pan_angle,  " to look"
        positions = {"head_tilt_joint": -0.65,
                     "head_pan_joint": pan_angle}
        if object_height is not None:

            positions["arm_lift_joint"] = object_height + 0.2
        self.body.move_to_joint_positions(positions)


