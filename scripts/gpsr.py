#!/usr/bin/env python

"""
Task 2 as specified in 5.3.3 of the Rulebook
"""

import rospy
from hsrb_interface import Robot
from villa_task.task3.commandanswerer import AnswerBase
from villa_task import tts

def detect_start_signal(wrench):
    force_torque_sensor = wrench
    while not rospy.is_shutdown():
        reading = force_torque_sensor.wrench
        if reading[0][0] > 5.0:
            break

     
def act(robot):
    
    #get speech interpreter
    answerhandler = AnswerBase()
    speech = tts.TextToSpeech()
    wrench = robot.get('wrist_wrench')
    # Block until we get told to start the test ########
    print("Waiting to start task...")
    detect_start_signal(wrench)
    print("Starting task now")
    #check continously to see if the robot should answer
    while not rospy.is_shutdown():
        #if the speech handler has heard something new
        if answerhandler.is_unanswered():
            #pull out the arguments from speech. If not understood, will return ("", "")
            arguments = answerhandler.answer(speech)
            print(arguments["action"])
            if arguments["action"] == "speak()":
                if arguments["person"] in ["asker", "person", None]:
                    answerhandler.set_speech_time(rospy.get_time())
                    speech.say(arguments["speech"])
                    answerhandler.set_speech_time(rospy.get_time())
                else:
                    answerhandler.set_speech_time(rospy.get_time())
                    speech.say("Sorry, I can't find " + arguments["person"])
                    answerhandler.set_speech_time(rospy.get_time())
                    speech.say("But I can say what you want.")
                    answerhandler.set_speech_time(rospy.get_time())
                    speech.say(arguments["speech"])
                    answerhandler.set_speech_time(rospy.get_time())
            else:
                answerhandler.set_speech_time(rospy.get_time())
                speech.say("I'm sorry, I can't do that.")
                answerhandler.set_speech_time(rospy.get_time())
                speech.say("I wish I could help!")
                answerhandler.set_speech_time(rospy.get_time())
                
                


with Robot() as robot:
    act(robot)