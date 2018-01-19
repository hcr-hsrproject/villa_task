#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from spr_qa.parse_commands import CommandParser
import csv
import os
import rospkg

class AnswerBase():
    def __init__(self):
        self.listen_time = 0
        self.speech_transcript = ''
        self.question_count = 0
        self.unanswered = False
        self.speech_time = 0
        self.rp = rospkg.RosPack()
        self.data_path = os.path.join(self.rp.get_path("villa_task"),"src/villa_task/spr/command_save.csv")
        
        print('Subscribing to speech detection..')
        speech_detection = rospy.Subscriber('/villa/speech_transcripts', String, self.__update_speech)
    
    def __update_speech(self, s):
        if rospy.get_time() - self.speech_time > 4:
            self.listen_time = rospy.get_time()
            self.speech_transcript = s.data
            data_dict = {}
            data_dict["timestamp"] = rospy.get_time()
            fieldnames = ["timestamp", "command"]
            print("--- HEARD: %s ---" % s.data)
            with open(self.data_path, 'a') as command_file:
                wr = csv.DictWriter(command_file, fieldnames=fieldnames)
                data_dict["command"] = '\"' + s.data + '\"'
                wr.writerow(data_dict)
                self.question_count = self.question_count + 1
           
            self.unanswered = True
            

    def answer(self, speech):
        c = CommandParser()
        arguments = c.parse(self.speech_transcript)
        if (arguments["action"] == None):
            answerstr = "Please repeat the command"
        else:
            answerstr = "You asked, " + self.speech_transcript
        if self.unanswered:
            self.speech_time = rospy.get_time()
            speech.say(answerstr, wait=True)
            self.speech_time = rospy.get_time()
            self.unanswered = False
        return arguments
        
    def is_unanswered(self):
        return self.unanswered
        
    def get_question_count(self):
        return self.question_count
        
    def get_listen_time(self):
        return self.listen_time
       
    def set_speech_time(self, setting):
        self.speech_time = setting
