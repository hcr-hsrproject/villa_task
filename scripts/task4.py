#!/usr/bin/env python

"""
Task 4 as specified in 5.3.3 of the Rulebook
"""

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from tmc_msgs.msg import Voice
import time
import math
import functools
import random
from hsrb_interface import Robot
from villa_task import tts
from villa_sound_localization.motionhandler import MotionHandler
from villa_task.task3.personangle import LocatePerson
from villa_task.task3.headcontrol import HeadControl
from villa_task.task3.crowdsurvey import ScanCrowd
from villa_task.task3.questionanswerer import AnswerBase
from villa_helpers.fast_move import FastMove
import re
from hsrb_interface import geometry

answerhandler = None
turning = False

def robot_moved(fastmove, noise_stop, whole_body, speech, motionhandler):
    global turning
    global answerhandler
    motionhandler.stop()
    detect_person = LocatePerson(fastmove)
    rospy.sleep(1.5)
    detect_person.turn_to_person()
    answerhandler.answer(speech)
    print("Restarting motion handler")
    motionhandler.start()
    turning = False

def begin_move(noise_start, motionhandler):
    global turning
    global answerhandler
    motionhandler.stop()
    rospy.sleep(2)
    motionhandler.start()
    # If time is less than 5 then move
    if abs(rospy.get_time() - answerhandler.get_listen_time()) < 5 and answerhandler.get_question_count() > 5:
        print("ALLOWING MOVE")
        turning = True
        return True
    else:
        if(answerhandler.get_question_count <= 5):
            print("STOPPED DUE TO RIDDLE GAME")
        else:
            print("STOPPED DUE TO SPEECH")
        return False
        


def act(robot):
    global answerhandler
    global turning
    
    #setup publishers and subscribers
    whole_body = robot.get('whole_body')
    omni_base = robot.get('omni_base')
    fastmove = FastMove(omni_base)
    speech = tts.TextToSpeech()
    noise_start = rospy.Publisher('/villa/sound_localization/noise/start', Bool, queue_size=1)
    noise_stop = rospy.Publisher('/villa/sound_localization/noise/stop', Bool, queue_size=1)
    
    #move robot body to a starting position
    whole_body.move_to_neutral()
    
    #setup the motion handling code
    crowd_watcher = ScanCrowd()
    head_status = HeadControl()
    answerhandler = AnswerBase()
    
    # Start by having the robot say it wants to play
    answerhandler.set_speech_time(rospy.get_time())
    speech.say("Let's play a riddle game", wait=True)
    answerhandler.set_speech_time(rospy.get_time())

    # Wait for people to get in position and then turn
    rospy.sleep(10)

    # Turn 180
    head_status.stop()
    fastmove.go(0., 0., math.pi, 100., relative=True)
    rospy.sleep(1.0)
    motionhandler = MotionHandler(fastmove, start_motion_callback=functools.partial(begin_move, noise_start), end_motion_callback=functools.partial(robot_moved, fastmove, noise_stop, whole_body, speech))
    #count the number of people, men, and women in the crowd
    answerhandler.set_speech_time(rospy.get_time())
    crowd_watcher.count_genders(speech)
    answerhandler.set_speech_time(rospy.get_time())
    rospy.sleep(0.5) # natural sounding pause
    answerhandler.set_speech_time(rospy.get_time())
    speech.say("Who wants to play riddles with me?", wait=True)
    answerhandler.set_speech_time(rospy.get_time())
    rospy.sleep(0.5)
    
    # Start listening to localization
    print("Starting motion handler")
    motionhandler.start()
    
    #check continously to see if the robot should answer
    while not rospy.is_shutdown():
        if answerhandler.get_question_count() <= 5 and answerhandler.is_unanswered():
            answerhandler.answer(speech)
        elif answerhandler.is_unanswered() == True and rospy.get_time() - answerhandler.get_listen_time() > 9.5 and not turning:
            print("Answering because HARK took too long")
            answerhandler.answer(speech)
        """if answerhandler.get_question_count() >= 15 and not answerhandler.is_unanswered():
            print("Ending because games are over")
            rospy.sleep(0.5)
            answerhandler.set_speech_time(rospy.get_time())
            speech.say("Thank you for playing a riddle game with me.", wait=True)
            answerhandler.set_speech_time(rospy.get_time())
            rospy.sleep(0.5)
            motionhandler.stop()
            break
            #EXIT THE ROOM HERE"""

with Robot() as robot:
    act(robot)

