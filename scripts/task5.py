#!/usr/bin/python
import rospy
from hsrb_interface import Robot, geometry, exceptions
from villa_task.task5 import motion
from villa_task.task5.detection_3d import Detection3DManager
from villa_task.task5.pick_and_place import grasp_any_object, place_anywhere
from villa_task.task5.octomap import OctomapManager
from villa_task.task5.collision_world_manager import CollisionWorldManager
from villa_task.task5.transforms import TransformManager
from villa_task.task5.speak import robot_say
from villa_task.task5.world_model import WorldModel
from villa_task.task5.motion import MotionManager
from villa_task.task5.perception import PerceptionManager
from villa_helpers.fast_move import FastMove

def perceive_table(transform_manager, world_model, perception_manager, motion_manager):
    # clear world model
    world_model.objects = None
    # look at table
    table_to_base = transform_manager.get_transform('base_link', 'table')
    motion_manager.look_at(table_to_base, world_model.table.pose.position.z)

    # segment for objects on table
    objects = perception_manager.plane_object_segmentation.service_call(table)

    #TODO: Add a fallback behavior
    if objects is None or len(objects) == 0:
        rospy.logerr("No objects found")
        exit(1)

    for object_name, obj in objects.items():
        new_relation = collision_manager.add_table_object(obj, object_name)
        transform_manager.add_relation(new_relation)

    transform_manager.broadcast_links()
    world_model.add_table_objects(objects)

tts = whole_body = omni_base = gripper = collision_world = wrench = None

while not rospy.is_shutdown():
    try:
        # Preparation to use robot functions
        rospy.loginfo("Obtaining robot resources...")
        robot = Robot()
        wrench = robot.get('wrist_wrench')
        tts = robot.try_get('default_tts')
        whole_body = robot.try_get('whole_body')
        omni_base = robot.try_get('omni_base')
        gripper = robot.try_get('gripper')
        collision_world = robot.try_get('global_collision_world')
        # Ensure that we have a clean slate
        collision_world.remove_all()
        tts.language = tts.ENGLISH
        break
    except (exceptions.ResourceNotFoundError,
            exceptions.RobotConnectionError) as e:
        rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

# Fast move
fm = FastMove(omni_base)

if __name__== '__main__':

    ##################
    # Initialization #
    ##################

    whole_body.move_to_go()
    rospy.loginfo("Waiting for services...")
    octomap_manager = collision_manager = transform_manager = landmark_detector = motion_manager = perception_manager = world_model = None
    while not rospy.is_shutdown():
        try:
            octomap_manager = OctomapManager()
            collision_manager = CollisionWorldManager(collision_world)
            transform_manager = TransformManager(whole_body)
            landmark_detector = Detection3DManager()
            motion_manager = MotionManager(omni_base, whole_body, fm, gripper, wrench)
            motion_manager.body.collision_world = collision_manager._world
            perception_manager = PerceptionManager()
            world_model = WorldModel()
            break
        except rospy.exceptions.ROSException as e:
            print(e)
            rospy.logerr("Failed to obtain services. Retrying...")

    rospy.loginfo("Services ready")

    rospy.loginfo("Waiting for start signal")

    # Block until we get told to start the test
    motion_manager.detect_start_signal()

    tts.say("Please open the cupboard")
    # scan to detect table
    motion_manager.head_scan_table()

    octomap_manager.get_octomap_cloud()
    table, table_relation = landmark_detector.detect_table(octomap_manager.octomap_cloud)
    world_model.add_table(table)

    transform_manager.add_relation(table_relation)
    transform_manager.broadcast_links()
    
    collision_manager.add_table(table)

    motion_manager.move_to_table(table, transform_manager)
    perceive_table(transform_manager, world_model, perception_manager, motion_manager)

    # We want to get the cupboard into the octomap
    motion_manager.look_backwards()
    # grasp first object
    object_name, object = None, None
    while object_name is None and not rospy.is_shutdown():
        object_name, object = grasp_any_object(transform_manager, motion_manager, collision_manager, world_model)

    world_model.picked_up_object(object_name)

    # scan to detect shelves
    octomap_manager.get_octomap_cloud()
    shelves, shelf_relations = landmark_detector.detect_shelf(octomap_manager.octomap_cloud)
    transform_manager.add_relations(shelf_relations)
    world_model.add_shelves(shelves)
    
    transform_manager.broadcast_links()
        
    collision_manager.add_cupboard(shelves)
    placed = place_anywhere(world_model, transform_manager, motion_manager, collision_manager, perception_manager)
    if not placed:
        exit(1)

    while len(world_model.objects) > 0 and not rospy.is_shutdown():
        object_name, object = grasp_any_object(transform_manager, motion_manager, collision_manager, world_model)
        if object_name is None:
            perceive_table(transform_manager, world_model, perception_manager, motion_manager)
            continue
        world_model.picked_up_object(object_name)
        placed = place_anywhere(world_model, transform_manager, motion_manager, collision_manager, perception_manager)
        if not placed:
            exit(1)

    collision_manager._world.remove_all()
