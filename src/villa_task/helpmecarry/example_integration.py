#!/usr/bin/python
# -*- coding: utf-8 -*-

from hsrb_interface import Robot
from hsrb_interface import geometry
from hsrb_interface import collision_world
from time import sleep

from villa_manipulation import bwi_perception_module, basic_grasp_planner
import sys
import rospy, roslib
import math, random

import numpy as np

import tf, tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

from visualization_msgs.msg import Marker, MarkerArray

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from villa_surface_detectors.srv import HorizontalPlanesDetect, HorizontalPlanesDetectRequest, ShelfDetect, ShelfDetectRequest, TableDetect, TableDetectRequest
from villa_manipulation.srv import object_frame_data, object_frame_dataRequest
from villa_helpers.fast_move import FastMove

# Preparation to use robot functions
robot = Robot()
                                        
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')
omni_base = robot.try_get('omni_base')
gripper = robot.try_get('gripper')
collision_world = robot.try_get('global_collision_world')
tts.language = tts.ENGLISH

# Fast move
fm = FastMove(omni_base)

bool_ready_to_process_octomap = False
bool_pointcloud_received = False
octomap_cloud = None
occupied_cells = None
    

def robot_say(string):
    print string
    #tts.say(string)

def get_transform(target, source):
    print "Waiting for transform from ",source,"to ",target, "..."
    while True:
        try:
            transform = whole_body._tf2_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(1.0))
            break
        except:
            continue
    return transform

def head_scan_table(): # scan table
    delay = 1
    whole_body.move_to_joint_positions({"arm_lift_joint":0.35, "arm_flex_joint":-0.5, "head_tilt_joint":-0.35, "head_pan_joint":-2.35})
    rospy.sleep(delay/2)
    whole_body.move_to_joint_positions({"head_tilt_joint":-0.65, "head_pan_joint":-2.35})    
    rospy.sleep(delay)
    whole_body.move_to_joint_positions({"head_tilt_joint":-0.65, "head_pan_joint":-3.24})
    rospy.sleep(delay)
    whole_body.move_to_joint_positions({"head_tilt_joint":-0.65, "head_pan_joint":-3.64})                
    rospy.sleep(delay/2)

def head_scan_shelf(table_transform):
    print "turning against the table"
    curr_pose = omni_base.pose
    print curr_pose
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
    
    pt = tf2_geometry_msgs.do_transform_pose(ps, table_transform)
           
    dist = (pt.pose.position.x, pt.pose.position.y)
    print dist

    theta = 0
    if abs(dist[1]) > abs(dist[0]) :
        print " aligning to y axis"
        if dist[1] < 0: 
            delta_x = 0
            delta_y = -table_ydim/2-0.6
            theta = -1.57 #3.14
        else:
            delta_x = 0
            delta_y = table_ydim/2+0.6
            theta = 1.57
    else: 
        print " aligning to x axis"
        if dist[0] < 0: 
            delta_x = -table_xdim/2-0.6
            delta_y = 0
            theta = -3.14 #1.57
        else:
            delta_x = table_xdim/2+0.6
            delta_y = 0
            theta = 0 #-1.57
            
    print "delta:",delta_x,",",delta_y," theta: ", theta  
    try:
        fm.move(geometry.pose(x=delta_x, y=delta_y, z=-table.pose.position.z), ref_frame_id='table')
    except:
        "Exception occurred"
        pass
    
    delay = 0.5
    whole_body.move_to_joint_positions({"head_tilt_joint":0.25, "head_pan_joint":theta})
    rospy.sleep(delay/2)
    whole_body.move_to_joint_positions({"head_tilt_joint":0.0}) 
    rospy.sleep(delay/2)    
    whole_body.move_to_joint_positions({"head_tilt_joint":0.35})    
    rospy.sleep(delay)
    whole_body.move_to_joint_positions({"head_tilt_joint":-0.65})
    rospy.sleep(delay)
    
def add_collision_using_ocotomap(collision_objects, min_height, max_height):
    global collision_world
    gen = pc2.read_points(octomap_cloud, skip_nans=True, field_names=("x","y","z"))
    ct = 0
    for pt in gen:
        if pt[2] > min_height:
            ct += 1
            collision_objects["octo"+str(ct)] = collision_world.add_box(x=0.02, y=0.02, z=0.02, pose=geometry.pose(x=pt[0], y=pt[1], z=pt[2]), frame_id='map')
    print "added ",ct, "cubes"

def octomap_cb(data):
    global bool_ready_to_process_octomap
    global bool_pointcloud_received
    global octomap_cloud    
    if ((bool_ready_to_process_octomap) and not(bool_pointcloud_received)):
        robot_say("Ready to process octomap cloud")
        bool_pointcloud_received = True
        octomap_cloud = data
 
def add_collision_table(collision_objects, table, table_xdim, table_ydim):
    global collision_world
    collision_world.remove_all()
    rot = tf.transformations.euler_from_quaternion([table.pose.orientation.x,
                                                    table.pose.orientation.y,
                                                    table.pose.orientation.z,
                                                    table.pose.orientation.w])
    collision_objects["table"] = collision_world.add_box(x=table_xdim, y=table_ydim, z=table.pose.position.z+0.02, pose=geometry.pose(x=table.pose.position.x, y=table.pose.position.y, z=table.pose.position.z/2+0.01,ei=rot[0], ej=rot[1], ek=rot[2]), frame_id='map')
    
    
def add_collision_shelf(collision_objects, shelves, selected_shelf, shelf_width, shelf_length):
    global collision_world
    rot = tf.transformations.euler_from_quaternion([shelves[selected_shelf].pose.orientation.x,
                                                    shelves[selected_shelf].pose.orientation.y,
                                                    shelves[selected_shelf].pose.orientation.z,
                                                    shelves[selected_shelf].pose.orientation.w])
    for shelf in shelves:
        collision_world.add_box(x=shelf.scale.x, y=shelf.scale.y, z=shelf.scale.z+0.02, pose=geometry.pose(x=shelf.pose.position.x, y=shelf.pose.position.y, z=shelf.pose.position.z,ei=rot[0], ej=rot[1], ek=rot[2]), frame_id='map')
    
    transform = get_transform('map', 'shelf'+str(selected_shelf))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'shelf'+str(selected_shelf)
    if shelf_width > shelf_length:
        x = 0.05
        y = shelf_length
        collision_world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x= -shelf_width/2, y=0, z=0.9-shelves[selected_shelf].pose.position.z), frame_id='shelf'+str(selected_shelf))
        collision_world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x= shelf_width/2, y=0, z=0.9-shelves[selected_shelf].pose.position.z), frame_id='shelf'+str(selected_shelf))
    else:
        x = shelf_width
        y = 0.05
        collision_world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x=0, y=-shelf_length/2, z=0.9-shelves[selected_shelf].pose.position.z), frame_id='shelf'+str(selected_shelf))
        collision_world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x=0, y=shelf_length/2, z=0.9-shelves[selected_shelf].pose.position.z), frame_id='shelf'+str(selected_shelf))


def openGripper(obj_width):
    if obj_width < 0.032:
        return 0.3
    elif obj_width < 0.042:
        return 0.4
    elif obj_width < 0.062:
        return 0.6
    elif obj_width < 0.092:
        return 0.8
    elif obj_width < 0.112:
        return 1.0
    else:
        return 1.2
   

if __name__=='__main__':

    ##################
    # Initialization #
    ##################
    
    try:
        robot_say("Waiting for services...")
        rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, octomap_cb)
        rospy.wait_for_service('horizontal_plane_detector_server')
        rospy.wait_for_service('cupboard_detector_service')    
        cupboard_detect_client = rospy.ServiceProxy("cupboard_detector_service", ShelfDetect)
        rospy.wait_for_service('table_detector_service')        
        table_detect_client = rospy.ServiceProxy("table_detector_service", TableDetect)
        tf_frames_client = rospy.ServiceProxy("object_frames", object_frame_data)
        rospy.wait_for_service('object_frames')
        robot_say("Services Ready")
        whole_body.move_to_go()
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
        
    par_frames = []
    child_frames = []
    poses = []
    collision_objects = {}
    initial_pose = omni_base.pose

    ########################
    # scan to detect table #
    ########################
        
    head_scan_table()
    rospy.sleep(1.0)
    
    bool_ready_to_process_octomap = True
    
    while (octomap_cloud == None):
        continue

    table = None
    table_xdim = 0
    table_ydim = 0
    
    try:
        robot_say("Extracting Table from Point Cloud Data")
        table_detect_response = table_detect_client(octomap_cloud)
        robot_say( "Extracted Table ") 
        talbe_box = table_detect_response.table_bounding_box
        table_xdim = talbe_box.scale.x
        table_ydim = talbe_box.scale.y
        print "table pose: \n",table_detect_response.table_bounding_box.pose
        table = table_detect_response.table_bounding_box
        par_frames.append('map')
        child_frames.append('table')
        poses.append(table.pose)
    except rospy.ServiceException as e:
        rospy.logerr(e)
        robot_say("Could not extract table point cloud") 
        sys.exit(1)
    
    try:
        robot_say("Broadcasting links")
        tf_frames_request = object_frame_dataRequest()
        tf_frames_request.parent_frames = par_frames
        tf_frames_request.child_frames = child_frames
        tf_frames_request.poses = poses
        tf_res = tf_frames_client(tf_frames_request)
        print "tf response: ", tf_res.success
    except rospy.ServiceException as e:
        rospy.logerr(e)
    
    add_collision_table(collision_objects, table, table_xdim, table_ydim)
    #add_collision_using_ocotomap(collision_objects,0.5,1.0)
    
    #################
    # look at table #
    #################
    
    pf = bwi_perception_module.BWISegmentation()
    
    table_to_base = get_transform('base_link','table')
    alpha = math.atan2(table_to_base.transform.translation.y,table_to_base.transform.translation.x)
    if alpha > 1.75:
        alpha -= 2*3.14
    print "Turning ", alpha,  " to look at table"
    try:
        whole_body.move_to_joint_positions({"head_tilt_joint":-0.65, "head_pan_joint": alpha})
    except:
        pass

    # segment for objects on table     
    pf.service_call(table) 
    
    if len(pf.objects) == 0:
        print "ERROR! No objects found!"
        exit(1)    
        
    for obj in pf.objects:
        print " - adding collision for ", obj
        rot = tf.transformations.euler_from_quaternion([pf.objects[obj].bbox.pose.orientation.x,
                                                    pf.objects[obj].bbox.pose.orientation.y,
                                                    pf.objects[obj].bbox.pose.orientation.z,
                                                    pf.objects[obj].bbox.pose.orientation.w])
        collision_objects[obj] = collision_world.add_box(x=pf.objects[obj].width*0.75, y=pf.objects[obj].length*0.75, z=pf.objects[obj].height, pose=geometry.pose(x=pf.objects[obj].x, y=pf.objects[obj].y, z=pf.objects[obj].z, ei=rot[0], ej=rot[1], ek=rot[2]), frame_id='table')
        par_frames.append('table')
        child_frames.append(obj)
        poses.append(pf.objects[obj].bbox.pose)
    
    try:
        robot_say("Broadcasting links")
        tf_frames_request = object_frame_dataRequest()
        tf_frames_request.parent_frames = par_frames
        tf_frames_request.child_frames = child_frames
        tf_frames_request.poses = poses
        tf_res = tf_frames_client(tf_frames_request)
        print "tf response: ", tf_res.success
    except rospy.ServiceException as e:
        rospy.logerr(e)
        
    ########################
    # grasp first object   #
    ########################
    
    
    table_transform = get_transform('table', 'map')
    
    print "moving to the table"
    curr_pose = omni_base.pose
    print curr_pose
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
    
    pt = tf2_geometry_msgs.do_transform_pose(ps, table_transform)
           
    dist = (pt.pose.position.x, pt.pose.position.y)
    print dist
    
    theta = 0
    aligned_axis = "x"
    
    if abs(dist[1]) > abs(dist[0]) :
        print " aligning to y axis"
        aligned_axis = "y"
        if dist[1] < 0: 
            delta_x = 0
            delta_y = -table_ydim/2-0.6
            theta = 1.57
        else:
            delta_x = 0
            delta_y = table_ydim/2+0.6
            theta = -1.57
    else: 
        print " aligning to x axis"
        aligned_axis = "x"
        if dist[0] < 0: 
            delta_x = -table_xdim/2-0.6
            delta_y = 0
            theta = 0
        else:
            delta_x = table_xdim/2+0.6
            delta_y = 0
            theta = 3.14
            
    print "delta:",delta_x,",",delta_y," theta: ", theta  
    try:
        fm.move(geometry.pose(x=delta_x, y=delta_y, z=-table.pose.position.z, ek = theta),8, ref_frame_id='table')
    except:
        "Exception occurred while moving to table"
        pass
        
    whole_body.collision_world = collision_world
    whole_body.end_effector_frame = u'hand_palm_link'
    whole_body.move_to_joint_positions({"head_tilt_joint":0.25, "arm_lift_joint":0.35, "head_pan_joint":-3.34}) 
    whole_body.move_to_joint_positions({"head_tilt_joint":-0.55, "head_pan_joint":-2.94}) 
      
    whole_body.move_to_joint_positions({"head_tilt_joint":-0.3, "head_pan_joint":-3.14}) 
   
    pf.sort_objects(aligned_axis) 
        
    obj_ct = 0
    transform = get_transform('base_link', 'table')
    while len(pf.sorted_objects) > obj_ct:
    
        g_planner = basic_grasp_planner.BasicGraspPlanner()
        grasp_obj_approach = pf.get_object_to_grasp(obj_ct, dist)
        
        selected_object = grasp_obj_approach[0] #pf.sorted_objects[obj_ct][0]
        approach = grasp_obj_approach[1]
        
        print "selected:", selected_object
        obj = selected_object
        
        
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'table'
        ps.pose.position.x = pf.objects[obj].x
        ps.pose.position.y = pf.objects[obj].y
        ps.pose.position.z = pf.objects[obj].z
        ps.pose.orientation.w = 1.0
        ps = tf2_geometry_msgs.do_transform_pose(ps, transform)

        pf.objects[obj].x = ps.pose.position.x
        pf.objects[obj].y = ps.pose.position.y
        pf.objects[obj].z = ps.pose.position.z
        
        # Grasp the object
        # patt = g_planner.getGrasp(pf.objects[selected_object], graspDirection=approach)
        
        obj_transform = get_transform(selected_object, 'map')
        patt = g_planner.getGraspWithPose(pf.objects[selected_object], omni_base.pose, obj_transform, aligned_axis, graspDirection=approach)
        if aligned_axis == "y":
            obj_width = pf.objects[selected_object].width
            obj_length = pf.objects[selected_object].length
        else:
            obj_width = pf.objects[selected_object].length
            obj_length = pf.objects[selected_object].width
        gripper.command(openGripper(obj_width))
        
        try:
            print "grasp pattern: ", patt
            whole_body.move_end_effector_pose(patt, selected_object)
            collision_world.remove(collision_objects[selected_object])
            try:
                whole_body.move_end_effector_by_line((0,0,1), 0.03)
            except:
                pass
            gripper.grasp(-0.02)
            whole_body.move_end_effector_by_line((1,0,0), 0.05)
            whole_body.move_end_effector_by_line((0,0,1), -0.2)
           
            print whole_body.joint_positions['hand_motor_joint'] 
            if whole_body.joint_positions['hand_motor_joint'] <= -0.7:
                print "Failed grasp for obj", obj_ct
                try:
                    fm.move(geometry.pose(x=delta_x, y=delta_y, z=-table.pose.position.z, ek = theta),8, ref_frame_id='table')
                except:
                    "Exception occurred while moving to table"
                    pass
                obj_ct += 1
                continue
            break
        except:
            print "ERROR! Cannot execute grasp for obj", obj_ct
            
            obj_ct += 1
            continue        
        
    whole_body.move_to_go()
    
    bool_ready_to_process_octomap = False
    bool_pointcloud_received = False
    octomap_cloud = None

    
    ##########################
    # scan to detect shelves #
    ##########################


    bool_ready_to_process_octomap = True

    while (octomap_cloud == None):
        continue

    shelves = []
    selected_shelf = 0
    shelf_width = 0
    shelf_length = 0
    try:
        shelf_detect_response = cupboard_detect_client(octomap_cloud)
        success_say = "Extracted " + str(len(shelf_detect_response.shelves)) +  " Shelves"
        idx  = 0
        for shelf in shelf_detect_response.shelf_bounding_boxes:
            if shelf.pose.position.z > 1.7:
                break
            shelves.append(shelf)
            par_frames.append('map')
            child_frames.append('shelf'+str(idx))
            poses.append(shelf.pose)
            idx += 1
        selected_shelf = random.randint(0,len(shelves)-1)
        shelf_width = shelves[selected_shelf].scale.x
        shelf_length = shelves[selected_shelf].scale.y
        print "selected shelf: ",selected_shelf, ",(w,l)", shelf_width, shelf_length
        print shelves[selected_shelf].pose
        robot_say(success_say)     
          
    except rospy.ServiceException as e:
        rospy.logerr(e)
        robot_say("Could not extract shelf point cloud")
    
    try:
        robot_say("Broadcasting links")
        tf_frames_request = object_frame_dataRequest()
        tf_frames_request.parent_frames = par_frames
        tf_frames_request.child_frames = child_frames
        tf_frames_request.poses = poses
        tf_res = tf_frames_client(tf_frames_request)
        print "tf response: ", tf_res.success
    except rospy.ServiceException as e:
        rospy.logerr(e)
        
    add_collision_shelf(collision_objects, shelves, selected_shelf, shelf_width, shelf_length)
    
    
    ###########################
    # iterate through objects #
    ###########################
    
    while len(pf.sorted_objects) > obj_ct:
        # put down object
        # whole_body.move_to_neutral()
        success = False
        tested_shelf = []
        while not success:
            try:
                print "moving to the shelf"
                c1 = None
                c2 = None
                curr_pose = omni_base.pose
                print curr_pose
                ps = geometry_msgs.msg.PoseStamped()
                ps.header.stamp = rospy.Time.now()
                ps.header.frame_id = 'map'
                ps.pose.position.x = curr_pose[0]
                ps.pose.position.y = curr_pose[1]
                ps.pose.position.z = 0
                ps.pose.orientation.x = shelves[selected_shelf].pose.orientation.x
                ps.pose.orientation.y = shelves[selected_shelf].pose.orientation.y
                ps.pose.orientation.z = shelves[selected_shelf].pose.orientation.z
                ps.pose.orientation.w = shelves[selected_shelf].pose.orientation.w
                
                shelf_transform = get_transform('shelf'+str(selected_shelf), 'map')
                pt = tf2_geometry_msgs.do_transform_pose(ps, shelf_transform)
                       
                dist = (pt.pose.position.x, pt.pose.position.y)
                print dist

                theta = 0
                
                if abs(dist[1]) > abs(dist[0]) :
                    print " aligning to y axis"
                    if dist[1] < 0: 
                        delta_x = 0
                        delta_y = -table_ydim/2-0.6
                        theta = 1.57
                    else:
                        delta_x = 0
                        delta_y = table_ydim/2+0.6
                        theta = -1.57
                else: 
                    print " aligning to x axis"
                    if dist[0] < 0: 
                        delta_x = -table_xdim/2-0.6
                        delta_y = 0
                        theta = 0
                    else:
                        delta_x = table_xdim/2+0.6
                        delta_y = 0
                        theta = 3.14
                        
                print "delta:",delta_x,",",delta_y," theta: ", theta  
                try:
                    fm.move(geometry.pose(x=delta_x, y=delta_y, z=-shelves[selected_shelf].pose.position.z, ek=theta),8, ref_frame_id=('shelf'+str(selected_shelf)))
                except:
                    "Exception occurred while moving to shelf"
                    pass
                 
                transform = get_transform('base_link', 'shelf'+str(selected_shelf))
                inverse_transform = get_transform('shelf'+str(selected_shelf), 'base_link')
                
                print "selected shelf:", selected_shelf
                ps = geometry_msgs.msg.PoseStamped()
                ps.header.stamp = rospy.Time.now()
                ps.header.frame_id = 'shelf'+str(selected_shelf)
                
                thresh = obj_length + 0.08
                
                if shelf_width > shelf_length: # put along x axis
                    ps.pose.position.x = - shelf_width/2 + (shelf_width/5)*random.randint(1,4)
                    if inverse_transform.transform.translation.y > 0:
                        ps.pose.position.y = shelf_length/4 + thresh
                    else:   
                        ps.pose.position.y = - shelf_length/4 - thresh
                else:
                    if inverse_transform.transform.translation.x > 0:
                        ps.pose.position.x = shelf_width/4 + thresh
                    else:
                        ps.pose.position.x = -shelf_width/4 - thresh
                    ps.pose.position.y = - shelf_length/2 + (shelf_width/5)*random.randint(1,4)
                ps.pose.position.z = pf.objects[selected_object].height*2/3 + shelves[selected_shelf].scale.z/2 + 0.03
                ps.pose.orientation.w = 1.0
                ps = tf2_geometry_msgs.do_transform_pose(ps, transform)
                
                # Add temporary collisions to avoid hitting top and bottom of shelf
                c1 = collision_world.add_box(x=shelves[selected_shelf].scale.x, y=shelves[selected_shelf].scale.y, z=0.5, pose=geometry.pose(z=0.55), frame_id='shelf'+str(selected_shelf))
                c2 = collision_world.add_box(x=shelves[selected_shelf].scale.x, y=shelves[selected_shelf].scale.y, z=0.5, pose=geometry.pose(z=-0.25), frame_id='shelf'+str(selected_shelf))
                
                whole_body.move_end_effector_pose( geometry.pose(x=ps.pose.position.x-0.06, y=ps.pose.position.y, z=ps.pose.position.z, ei=3.14,ej=-1.57, ek=0.0), 'base_link')
                whole_body.move_end_effector_by_line((0,0,1), thresh)
               
                gripper.command(1.2)
                
                collision_world.remove(c1)
                collision_world.remove(c2)
                c1 = None
                c2 = None
                
                obj_ct += 1
                
                whole_body.move_end_effector_by_line((0,0,1), -thresh*2)
                whole_body.move_to_go()
                
                print "moving to the table"
                curr_pose = omni_base.pose
                print curr_pose
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
                
                pt = tf2_geometry_msgs.do_transform_pose(ps, table_transform)
                       
                dist = (pt.pose.position.x, pt.pose.position.y)
                print dist
                
                theta = 0
                if abs(dist[1]) > abs(dist[0]) :
                    print " aligning to y axis"
                    if dist[1] < 0: 
                        delta_x = 0
                        delta_y = -table_ydim/2-0.6
                        theta = 1.57
                    else:
                        delta_x = 0
                        delta_y = table_ydim/2+0.6
                        theta = -1.57
                else: 
                    print " aligning to x axis"
                    if dist[0] < 0: 
                        delta_x = -table_xdim/2-0.6
                        delta_y = 0
                        theta = 0
                    else:
                        delta_x = table_xdim/2+0.6
                        delta_y = 0
                        theta = 3.14
                        
                
                print "delta:",delta_x,",",delta_y," theta: ", theta  
                try:
                    fm.move(geometry.pose(x=delta_x, y=delta_y, z=-table.pose.position.z, ek=theta),8, ref_frame_id='table')
                except:
                    "Exception occurred"
                    pass
                
                success = True
            except Exception as e:
                rospy.logerr(e)
                if c1 != None:
                    collision_world.remove(c1)
                if c2 != None:
                    collision_world.remove(c2)
                tested_shelf.append(selected_shelf)
                if len(tested_shelf) == len(shelves):
                    sys.exit(1)
                new_shelf = random.randint(0,len(shelves)-1)
                while new_shelf in tested_shelf:
                    new_shelf = random.randint(0,len(shelves)-1)
                selected_shelf = new_shelf
               
                continue
            
        selected_shelf = random.randint(0,len(shelves)-1)
        print "waiting for transformation..."
            
        transform = get_transform('base_link', 'table')
        
        while len(pf.sorted_objects) > obj_ct:
            "Planning to grasp"
            g_planner = basic_grasp_planner.BasicGraspPlanner()
            grasp_obj_approach = pf.get_object_to_grasp(obj_ct, dist)
            
            selected_object = grasp_obj_approach[0] #pf.sorted_objects[obj_ct][0]
            approach = grasp_obj_approach[1]
            
            print "selected:", selected_object
            obj = selected_object
            ps = geometry_msgs.msg.PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = 'table'
            ps.pose.position.x = pf.objects[obj].x
            ps.pose.position.y = pf.objects[obj].y
            ps.pose.position.z = pf.objects[obj].z
            ps.pose.orientation.w = 1.0
            ps = tf2_geometry_msgs.do_transform_pose(ps, transform)

            pf.objects[obj].x = ps.pose.position.x
            pf.objects[obj].y = ps.pose.position.y
            pf.objects[obj].z = ps.pose.position.z
            
            # Grasp the object
            obj_transform = get_transform(selected_object, 'map')
            patt = g_planner.getGraspWithPose(pf.objects[selected_object], omni_base.pose, obj_transform, aligned_axis, graspDirection=approach)
            if aligned_axis == "y":
                obj_width = pf.objects[selected_object].width
                obj_length = pf.objects[selected_object].length
            else:
                obj_width = pf.objects[selected_object].length
                obj_length = pf.objects[selected_object].width
            gripper.command(openGripper(obj_width))
            
            try:
                print "grasp pattern: ", patt
                whole_body.move_end_effector_pose(patt, selected_object)
                collision_world.remove(collision_objects[selected_object])
                try:
                    whole_body.move_end_effector_by_line((0,0,1), 0.03)
                except:
                    pass
                gripper.grasp(-0.02)
                whole_body.move_end_effector_by_line((1,0,0), 0.05)
                whole_body.move_end_effector_by_line((0,0,1), -0.2)
               
                print whole_body.joint_positions['hand_motor_joint'] 
                if whole_body.joint_positions['hand_motor_joint'] <= -0.7:
                    print "Failed grasp for obj", obj_ct
                    try:
                        fm.move(geometry.pose(x=delta_x, y=delta_y, z=-table.pose.position.z, ek = theta),8, ref_frame_id='table')
                    except:
                        "Exception occurred while moving to table"
                        pass
                    obj_ct += 1
                    continue
                break
            except:
                print "ERROR! Cannot execute grasp for obj", obj_ct
                
                obj_ct += 1
                continue              
        
        
        
        whole_body.move_to_go()
    
    
    collision_world.remove_all()
    

    


