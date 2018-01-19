from villa_task.task5 import motion
import geometry_msgs.msg
import rospy
from villa_manipulation.basic_grasp_planner import BasicGraspPlanner
import tf2_geometry_msgs
from hsrb_interface import geometry

def grasp_object(selected_object_name, world_model, transform_manager, motion_manager, collision_manager, g_planner, aligned_axis, approach ):
    print "selected:", selected_object_name
    target_object = world_model.objects[selected_object_name]
    transform = transform_manager.get_transform('base_link', 'table')

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'table'
    ps.pose.position.x = target_object.x
    ps.pose.position.y = target_object.y
    ps.pose.position.z = target_object.z
    ps.pose.orientation.w = 1.0
    ps = tf2_geometry_msgs.do_transform_pose(ps, transform)

    target_object.x = ps.pose.position.x
    target_object.y = ps.pose.position.y
    target_object.z = ps.pose.position.z

    # Grasp the object
    obj_transform = transform_manager.get_transform(selected_object_name, 'map')
    patt = g_planner.getGraspWithPose(target_object, motion_manager.omnibase.pose, obj_transform, aligned_axis,
                                      graspDirection=approach)
    if aligned_axis == "y":
        obj_width = target_object.width
        obj_length = target_object.length
    else:
        obj_width = target_object.length
        obj_length = target_object.width

    try:
        print "grasp pattern: ", patt
        motion_manager.open_gripper(obj_width)
        motion_manager.body.move_end_effector_pose(patt, selected_object_name)
        collision_manager.remove(selected_object_name)
        try:
            motion_manager.body.move_end_effector_by_line((0, 0, 1), 0.03)
        except Exception as e:
            print "Could not move in line"
            raise
        motion_manager.gripper.grasp(-0.05)
        motion_manager.body.move_end_effector_by_line((1, 0, 0), 0.05)
        motion_manager.body.move_end_effector_by_line((0, 0, 1), -0.2)

        print motion_manager.body.joint_positions['hand_motor_joint']

        if motion_manager.grasp_failed():
            print "Failed grasp for ", selected_object_name
            return False
        else:
            return True

    except Exception as e:
        print "ERROR! Cannot execute grasp for ", selected_object_name
        print e
        return False


def grasp_any_object(transform_manager, motion_manager, collision_manager, world_model, max_num_tries=2):

    theta, aligned_axis, dist, delta_x, delta_y = motion_manager.move_to_table(world_model.table, transform_manager)

    world_model.set_axis(aligned_axis)

    sorted_objects = world_model.sort_objects(aligned_axis)

    object_index = 0

    have_object = False
    selected_object_name = None
    while len(sorted_objects) > object_index and not have_object and object_index < (max_num_tries - 1):
        g_planner = BasicGraspPlanner()
        selected_object_name = sorted_objects[object_index][0]
        approach = "side"

        have_object = grasp_object(selected_object_name, world_model, transform_manager, motion_manager, collision_manager, g_planner, aligned_axis, approach)
        object_index += 1

    motion_manager.body.move_to_go()
    if have_object:
        return selected_object_name, world_model.objects[selected_object_name]
    else:
        return None, None


def place_anywhere_on_shelf(world_model, transform_manager, motion_manager, collision_manager, shelf, shelf_name, placement):
        # put down object
        # whole_body.move_to_neutral()
        object_in_hand = world_model.current_object
        success = False
        #motion_manager.move_to_shelf(shelf, shelf_name, world_model, transform_manager)

        c1 = None
        c2 = None
        try:

            transform = transform_manager.get_transform('base_link', shelf_name)
            inverse_transform = transform_manager.get_transform(shelf_name, 'base_link')

            print "selected shelf:", shelf_name
            ps = geometry_msgs.msg.PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = shelf_name

            adjustment_factor = max(object_in_hand.length, object_in_hand.width) + 0.08

            # Add temporary collisions to avoid hitting top and bottom of shelf
            c1 = collision_manager._world.add_box(x=shelf.scale.x, y=shelf.scale.y,
                                         z=0.5, pose=geometry.pose(z=0.55), frame_id=shelf_name)
            c2 = collision_manager._world.add_box(x=shelf.scale.x, y=shelf.scale.y,
                                         z=0.5, pose=geometry.pose(z=-0.25), frame_id=shelf_name)

            ps.pose.position.x = placement[0]
            ps.pose.position.y = placement[1]
            ps.pose.position.z = object_in_hand.height / 2.0 + shelf.scale.z/ 2.0 + 0.04 #FIXME
            ps.pose.orientation.w = 1.0
            ps = tf2_geometry_msgs.do_transform_pose(ps, transform)

            preplace_pose = geometry.pose(x=ps.pose.position.x - 0.06, y=ps.pose.position.y, z=ps.pose.position.z, ei=3.14, ej=-1.57, ek=0.0)
            print preplace_pose
            motion_manager.body.move_end_effector_pose(preplace_pose, 'base_link')
            motion_manager.body.move_end_effector_by_line((0, 0, 1), adjustment_factor)

            motion_manager.open_gripper()

            collision_manager._world.remove(c1)
            collision_manager._world.remove(c2)
            c1 = None
            c2 = None

            motion_manager.body.move_end_effector_by_line((0, 0, 1), -adjustment_factor)
            motion_manager.body.move_to_go()

        except Exception as e:
            rospy.logerr(e)
            if c1 is not None:
                collision_manager._world.remove(c1)
            if c2 is not None:
                collision_manager._world.remove(c2)
            return False

        return True

def place_anywhere(world_model, transform_manager, motion_manager, collision_manager, perception_manager):
    # Try to put it on any shelf
    shelves_tried_so_far = set()
    placed = False
    while not placed:
        if len(shelves_tried_so_far) == len(world_model.shelves):
            print("Couldn't place object")
            exit(1)
        shelf_name = None
        while shelf_name is None or shelf_name in shelves_tried_so_far:
            selected_shelf, shelf_name = perception_manager.select_shelf(world_model)
        motion_manager.move_to_shelf(selected_shelf, shelf_name, transform_manager)
        inverse_transform = transform_manager.get_transform(shelf_name, 'base_link')
        placement = perception_manager.decide_placement(selected_shelf, [], world_model.current_object,
                                                        inverse_transform)
        object_placed = place_anywhere_on_shelf(world_model, transform_manager, motion_manager, collision_manager, selected_shelf, shelf_name, placement)
        placed = object_placed
        shelves_tried_so_far.add(shelf_name)
    return placed
