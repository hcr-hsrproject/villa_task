import rospy
from villa_sound_localization.motionhandler import MotionHandler
from villa_task.spr.personangle import LocatePerson

class MotionManager(object):

    def __init__(self, fastmove, whole_body, tts, answerhandler):
        self.fastmove = fastmove
        self.whole_body = whole_body
        self.tts = tts
        self.answerhandler = answerhandler
        self.detect_person = LocatePerson(fastmove)
        self.turning = False

        self.motionhandler = MotionHandler(fastmove, start_motion_callback=self.__begin_move, end_motion_callback=self.__robot_moved)

    # Waits for wrist to be tapped
    @staticmethod
    def wait_for_start_signal(wrench):
        force_torque_sensor = wrench
        while not rospy.is_shutdown():
            reading = force_torque_sensor.wrench
            if reading[0][0] > 5.0:
                break


    # Start underlying motionhandler
    def start(self):
        self.motionhandler.start()


    # Beginning of sloc motion callback
    # Return T/F determines whether to cancel
    def __begin_move(self, _):
        if self.answerhandler.get_question_count() < 5:
            rospy.loginfo("STOPPED DUE TO RIDDLE GAME")
            return False

        self.motionhandler.stop()

        timeout = rospy.Time.now() + rospy.Duration(3)
        while rospy.get_time() - self.answerhandler.get_listen_time() > 5 and rospy.get_time() < timeout:
            rospy.sleep(0.05)

        self.motionhandler.start()

        # If time is less than 5 then move
        if abs(rospy.get_time() - self.answerhandler.get_listen_time()) < 5:
            rospy.loginfo("Allowing Move")
            self.turning = True
            return True
        else:
            rospy.loginfo("STOPPED DUE TO SPEECH")
            return False


    # End of sloc motion callback
    def __robot_moved(self, _):
        self.motionhandler.stop()
        rospy.sleep(1.5)
        self.detect_person.turn_to_person()
        self.answerhandler.answer(self.tts)
        rospy.loginfo("Restarting motion handler")
        self.motionhandler.start()
        self.turning = False


