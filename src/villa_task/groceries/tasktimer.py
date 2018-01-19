import rospy
import tf2_ros

"""
Timer for the task
"""
class TaskTimer(object):
    SAFETY_THRESHOLD = 10 # seconds before time limit to execute end callbacks

    def __init__(self, timelimit, motion, world_model, objectlabeller, report_service):
        self.timelimit = rospy.Time.now() + rospy.Duration(timelimit)
        self.timer = rospy.Timer(rospy.Duration(5), self.__timer_cb)

        self.motion = motion
        self.world_model = world_model
        self.objectlabeller = objectlabeller
        self.report_service = report_service

    def __timer_cb(self, te):
        time_left = self.timelimit - te.current_real

        if time_left <= rospy.Duration(TaskTimer.SAFETY_THRESHOLD):
            self.__end_cb()

    def __end_cb(self):
        try:
            shelf_to_base = self.world_model.transform_manager.get_transform('base_link', 'shelf0')
        except tf2_ros.LookupException:
            rospy.logerr('No base_link to shelf0 transform!')
            return

        look_future = self.motion.look_at(shelf_to_base, self.world_model.shelves[0].bbox.pose)
        look_future(10)
        rospy.sleep(1.5) # stabilize image

        annotated_shelf = self.objectlabeller.annotate_image()
        # TODO: Add detected objects with images
        self.report_service(annotated_shelf, '', [])
