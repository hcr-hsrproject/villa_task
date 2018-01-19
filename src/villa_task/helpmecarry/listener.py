#!/usr/bin/env python
import sys
import rospy, roslib
# import std_msgs.msg
from std_msgs.msg import *
# from std_msgs.msg import String
# from std_msgs.msg import Bool
# from spr_qa.parse_commands import CommandParser
import csv
import os
import rospkg


class SpeechManager():
    def __init__(self):
        # holds last time utterance was heard
        self.listen_time = 0
        # holds last time robot spoke
        self.speech_time = 0
        # holds the last utterance heard
        self.speech_transcript = 'stop'

        # boolean value of whether robot should follow person or not
        self.heard_activate = False

        #
        self.heard_stop = False

        # contains name of person leading task
        self.person_name = None
        # contains location needed to go
        self.location = None
        self.rp = rospkg.RosPack()

        self.follow_cmd = std_msgs.msg.Bool()
        self.name_file = self.data_path = os.path.join(self.rp.get_path("spr_qa"), "src/spr_qa/names.txt")
        self.location_file = self.data_path = os.path.join(self.rp.get_path("spr_qa"), "src/spr_qa/locations.txt")

        # locations of predefined names, locations
        self.names = open(self.name_file, "r")
        self.locations = open(self.location_file, "r")
        # self.data_path = os.path.join(self.rp.get_path("villa_task"), "src/villa_task/help_me_carry/command_save.csv")

        speech_detection = rospy.Subscriber('/villa/speech_transcripts', String, self.__update_speech)


    def __update_speech(self, s):
        # only update if the robot hasn't spoken recently
        if rospy.get_time() - self.speech_time > 4:
            # set listen time
            self.listen_time = rospy.get_time()
            # collect utterance
            self.speech_transcript = s.data
            rospy.loginfo("Heard: " + self.speech_transcript)
            # find name in utterance
            for i in self.names:
                if i[:-1] in self.speech_transcript:
                    self.person_name = i[:-1]
            # find location in utterance
            for i in self.locations:
                if i[:-1] in self.speech_transcript:
                    self.location = i[:-1]

            # determine if person asked to follow or stop
            if "stop" in self.speech_transcript.lower():
                rospy.loginfo("I heard Stop")
                self.heard_stop = True

            else: #"follow" in self.speech_transcript.lower():
                rospy.loginfo("I heard follow")
                self.heard_activate = True

            # collect data on utterance
            data_dict = {}
            data_dict["timestamp"] = rospy.get_time()
            fieldnames = ["timestamp", "command"]
            print("--- HEARD: %s ---" % s.data)
            # with open(self.data_path, 'a') as command_file:
            #     wr = csv.DictWriter(command_file, fieldnames=fieldnames)
            #     data_dict["command"] = '\"' + s.data + '\"'
            #     wr.writerow(data_dict)


    def set_speech_time(self, setting):
        self.speech_time = setting

    def capture_operator_name(self):
        while not rospy.is_shutdown():
            if self.person_name is not None:
                name = self.person_name
                self.person_name = None
                return name

    def listen_for_stop(self):
        while not rospy.is_shutdown():
            if self.heard_stop:
                self.heard_stop = False
                return True

    def listen_for_activation(self):
        while not rospy.is_shutdown():
            if self.heard_activate:
                self.heard_activate = False
                return True
