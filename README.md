# Villa Task
This repo contains task demos for the Toyota HSR

## Groceries (Task 5)

Ensure that the robot is localized, or clear the map with

    configure_map blank

To run the task:

    roslaunch villa_task groceries.launch

Then tap the robot on the wrist to start.

### Starting the services and task separately

    roslaunch villa_task groceries.launch run_task_executor:=false

Now to run the executor

    ROS_NAMESPACE=/villa/groceries rosrun villa_task groceries /villa/groceries/robot_description:=/robot_description

## SPR (Task 4)

To run the task:

    roslaunch villa_task spr.launch

Then tap the robot on the wrist to start.

## Helpmecarry 
Ensure that the robot is localized, or clear the map with

    configure_map gdc4
    
Ensure that Yolo is turning on

    any yolo is running
    
Ensure that turning on the microphone

    rosrun villa_sound_localization audiostream_sharedmem.n
    
Launch helpmecarry (In the robot)
    
    roslaunch villa_task helpmecarry_robot.launch
    
 

