import geometry_msgs.msg
import rospy
import sensor_msgs
import tf2_geometry_msgs
from hsrb_interface import exceptions
from hsrb_interface import geometry
from villa_manipulation.basic_grasp_planner import BasicGraspPlanner

from villa_task.groceries import visualization
from villa_task.groceries.exceptions import PreGraspPlanFailure, MidGraspPlanFailure, ItemNotGraspedFailure


def grasp_object(selected_object_name, world_model, motion_manager, g_planner, in_base_frame=False):
    # How far from the object should we try to put the gripper before coming in for the grasp?
    DIST_FROM_OBJECT_SIDE = 0.08
    PULL_BACK_AMOUNT = DIST_FROM_OBJECT_SIDE * 2
    LIFT_AMOUNT = 0.05
    target_object = world_model.table_objects[selected_object_name]
    
    if in_base_frame:
        print "Planning in base frame"
    
        # transform object pose into base frame
        transform = world_model.transform_manager.get_transform('base_link', 'table')
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
        obj_transform = world_model.transform_manager.get_transform(selected_object_name, 'map')
        style, grasp_pattern, gripper_width = g_planner.get_grasp_for_pose_in_base_frame(target_object, motion_manager.omnibase.pose, obj_transform, motion_manager._align_to_table_axis, distance_from_object_side=DIST_FROM_OBJECT_SIDE)
        
        if grasp_pattern is None:
            raise PreGraspPlanFailure()

        try:
            print "grasp pattern: ", grasp_pattern
            visualization.publish_goal_pose(grasp_pattern, 'base_link')
            motion_manager.open_gripper(gripper_width)
            motion_manager.body.move_end_effector_pose(grasp_pattern, ref_frame_id='base_link')
        except exceptions.MotionPlanningError as e:
            rospy.loginfo("Failed to plan to pre-grasp pose")
            rospy.loginfo(e)
            raise PreGraspPlanFailure()

    else:
        print "Planning in object frame"
        # Grasp the object
        style, grasp_pattern, gripper_width = g_planner.get_grasp_for_pose(target_object, distance_from_object_side=DIST_FROM_OBJECT_SIDE)
        if grasp_pattern is None:
            raise PreGraspPlanFailure()
        try:
            print "grasp pattern: ", grasp_pattern
            visualization.publish_goal_pose(grasp_pattern, selected_object_name)
            motion_manager.open_gripper(gripper_width)
            motion_manager.body.move_end_effector_pose(grasp_pattern, ref_frame_id=selected_object_name)
        except exceptions.MotionPlanningError as e:
            rospy.loginfo("Failed to plan to pre-grasp pose")
            rospy.loginfo(e)
            raise PreGraspPlanFailure()

    world_model.disable_collision_for(selected_object_name)

    try:
        motion_manager.body.move_end_effector_by_line((0, 0, 1), DIST_FROM_OBJECT_SIDE)
    except exceptions.MotionPlanningError as e:
        world_model._add_table_object_collision(target_object, selected_object_name)
        rospy.loginfo(e)
        # TODO: Failing at this point is awkward, and probably avoidable if
        # we test a plan to this point before we stuck the hand out
        rospy.logerr("Failed to plan during grasp execution")
        raise MidGraspPlanFailure()
    
    motion_manager.gripper.grasp(-0.05)

    if not motion_manager.is_holding_something():
        rospy.logerr("Grasp failed")
        raise ItemNotGraspedFailure()

    try:
        # Axes wrt to palm link
        if style == "side":
            lift_axis = (1, 0, 0)
            pull_back_axis = (0, 0, -1)
        else:
            lift_axis = (0, 0, -1)
            pull_back_axis = (0, -1, 0)

        # Lift
        motion_manager.body.move_end_effector_by_line(lift_axis, LIFT_AMOUNT)

        # Pull back
        motion_manager.body.move_end_effector_by_line(pull_back_axis, PULL_BACK_AMOUNT)
    except exceptions.MotionPlanningError as e: # ?????????
        rospy.logerr("Failed to plan during retract")
        # If we have grasped an object but can't move by line back, then
        # let's carpe diem and unsafe_move_to_neutral. What could go wrong?
        # TODO: Test on high table
        #motion_manager.body.move_to_joint_positions({"head_pan_joint":0.0})
        motion_manager.groovy.unsafe_move_arm_to_neutral()

    return True


def grasp_any_object(motion_manager, world_model, max_num_tries=2):
    sorted_objects = world_model.sort_table_objects(motion_manager._align_to_table_axis, motion_manager.omnibase.pose)

    object_index = 0
    num_attempted_grasps = 0
    have_object = False
    selected_object_name = None
    g_planner = BasicGraspPlanner()
    while object_index < len(sorted_objects) and not have_object and num_attempted_grasps < max_num_tries:
        selected_object_name = sorted_objects[object_index][0]
        rospy.loginfo("Attempting to grasp {}".format(selected_object_name))
        try:
            have_object = grasp_object(selected_object_name, world_model, motion_manager, g_planner, in_base_frame=False)
        except PreGraspPlanFailure as e:
            rospy.logerr(e)
            object_index += 1
            continue
        except MidGraspPlanFailure as e:
            rospy.logerr(e)
            num_attempted_grasps += 1
            object_index += 1
            continue
        except ItemNotGraspedFailure as e:
            # If we closed the gripper and don't think we have an object, probably want to bubble up the failure
            rospy.logerr(e)
            num_attempted_grasps += 1
            object_index += 1

    if have_object:
        return selected_object_name, world_model.table_objects[selected_object_name]
    else:
        return None, None


def place_object_on_shelf(world_model, motion_manager, shelf, shelf_name, placement):
    object_in_hand = world_model.current_object
    # The distance between the object and the edge of the shelf once in pre-place pose
    DISTANCE_FROM_SHELF = 0.05
    HEIGHT_ABOVE_SHELF = 0.04

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = shelf_name
    # FIXME: Doesn't account for object dimensions correctly!
    ps.pose.position.x = - (shelf.scale.x / 2.0 + object_in_hand.height + DISTANCE_FROM_SHELF)
    ps.pose.position.y = placement[1]
    # TODO: Handle top grasp object dimension flip
    ps.pose.position.z = object_in_hand.height / 2.0 + shelf.scale.z / 2.0 + HEIGHT_ABOVE_SHELF
    preplace_pose = geometry.pose(x=ps.pose.position.x, y=ps.pose.position.y, z=ps.pose.position.z, ei=3.14, ej=-1.57,
                                  ek=0.0)


    # Add temporary collisions to avoid hitting top and bottom of shelf
    c1 = world_model.collision_world.add_box(x=shelf.scale.x, y=shelf.scale.y,
                                             z=0.5, pose=geometry.pose(z=0.55), frame_id=shelf_name)
    c2 = world_model.collision_world.add_box(x=shelf.scale.x, y=shelf.scale.y,
                                             z=0.5, pose=geometry.pose(z=-0.25), frame_id=shelf_name)

    try:
        visualization.publish_goal_pose(preplace_pose, shelf_name)
        motion_manager.body.move_end_effector_pose(preplace_pose, shelf_name)
    except exceptions.MotionPlanningError as e:
        rospy.loginfo("Failed to plan to pre-place pose")
        rospy.loginfo(e)
        world_model.collision_world.remove(c1)
        world_model.collision_world.remove(c2)
        return False

    distance_to_move = placement[0] - ps.pose.position.x
    try:
        motion_manager.body.move_end_effector_by_line((0, 0, 1), distance_to_move)
    except exceptions.MotionPlanningError as e:
        # TODO: Pretty awkward time to fail. Should test this before we stick the arm and the body up here
        rospy.logerr("Failed to plan moving item into the shelf")
        rospy.logerr(e)
        world_model.collision_world.remove(c1)
        world_model.collision_world.remove(c2)
        return False

    motion_manager.open_gripper()

    world_model.collision_world.remove(c1)
    world_model.collision_world.remove(c2)

    try:
        motion_manager.body.move_end_effector_by_line((0, 0, 1), -distance_to_move)
        motion_manager.groovy.unsafe_move_arm_to_neutral()
    except exceptions.MotionPlanningError as e:
        rospy.logerr("Failed to plan moving arm out of shelf")
        rospy.logerr(e)
        return False

    return True


def place_anywhere(world_model, motion_manager, perception_manager):
    # Try to put it on any shelf
    object_name = world_model.current_object
    shelves_tried_so_far = set()
    placed = False
    shelf_index = None
    untried_preferred_shelves = world_model.get_sorted_target_shelves_for_object(object_name)
    while not placed:

        if len(untried_preferred_shelves) > 0:
            shelf, shelf_index = untried_preferred_shelves[0]
            untried_preferred_shelves.remove((shelf, shelf_index))
        else:
            all_shelf_indices = list(range(0, len(world_model.shelves)))
            not_tried_shelves = set(all_shelf_indices) - shelves_tried_so_far
            if len(not_tried_shelves) == 0:
                rospy.logerr("Couldn't place object")
                # FIXME: How do we fallback here?
                exit(1)
            shelf_index = list(not_tried_shelves)[0]
            shelf = world_model.shelves[shelf_index].bbox

        shelf_name = "shelf" + str(shelf_index)
        rospy.loginfo("Attempting to place on {}".format(shelf_name))
        motion_manager.approach_cupboard()

        #possibilities = perception_manager.collision_mapping(shelf)
        possibilities = None
        if possibilities is not None and False:
            gen = sensor_msgs.point_cloud2.read_points(possibilities, skip_nans=True, field_names=("x", "y", "z"))

            a_placement_point = None
            for point in gen:
                a_placement_point = point
                if a_placement_point is not None:
                    break
            object_placed = place_object_on_shelf(world_model, motion_manager, shelf, shelf_name, a_placement_point)
        else:
            placement = perception_manager.decide_placement(shelf, world_model.current_object)
            rospy.loginfo("Attempting to place on {}".format(shelf_name))
            object_placed = place_object_on_shelf(world_model, motion_manager, shelf, shelf_name, placement)

        placed = object_placed
        shelves_tried_so_far.add(shelf_index)

    return placed, shelf_index
