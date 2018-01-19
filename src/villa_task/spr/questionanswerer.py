#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from spr_qa.xml_kbase import XMLKnowledgeBase
from spr_qa.spr_qa_main import QuestionAnswerer
from spr_qa.correct_utterance import CorrectUtterance
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
        self.template_filename = os.path.join(self.rp.get_path("spr_qa"),"src/spr_qa/resources/spr_qa_updated.csv") 
        self.location_xml_filename = os.path.join(self.rp.get_path("spr_qa"),"src/spr_qa/resources/xml_files/Locations_mod.xml")
        self.object_xml_filename = os.path.join(self.rp.get_path("spr_qa"),"src/spr_qa/resources/xml_files/Objects_mod.xml")
        self.corrector = CorrectUtterance()
        self.can_clarify = False
        self.data_path = os.path.join(self.rp.get_path("villa_task"),"src/villa_task/spr/question_save.csv")
        
        print('Subscribing to speech detection..')
        speech_detection = rospy.Subscriber('/villa/speech_transcripts', String, self.__update_speech)
    
    def __update_speech(self, s):
        if rospy.get_time() - self.speech_time > 4:
            self.listen_time = rospy.get_time()
            self.speech_transcript = self.corrector.correct(s.data)
            data_dict = {}
            data_dict["timestamp"] = rospy.get_time()
            fieldnames = ["timestamp", "command"]
            print("--- QUESTION COUNT: %d ---" % self.question_count)
            print("--- HEARD: %s ---" % s.data)
            if self.speech_transcript == None and self.question_count >= 5 and self.can_clarify == True:
                print("--- NO CORRECTION FOUND ---")
                with open(self.data_path, 'a') as command_file:
                    wr = csv.DictWriter(command_file, fieldnames=fieldnames)
                    data_dict["command"] = '\"' + s.data + '\"'
                    wr.writerow(data_dict)
            elif self.speech_transcript == None:
                self.question_count = self.question_count + 1
                print("--- NO CORRECTION FOUND ---")
                with open(self.data_path, 'a') as command_file:
                    wr = csv.DictWriter(command_file, fieldnames=fieldnames)
                    data_dict["command"] = '\"' + s.data + '\"'
                    wr.writerow(data_dict)
            else:
                self.question_count = self.question_count + 1
                print("--- CORRECTED: %s ---" % self.speech_transcript)
                with open(self.data_path, 'a') as command_file:
                    wr = csv.DictWriter(command_file, fieldnames=fieldnames)
                    data_dict["command"] =  '\"' + self.speech_transcript + '"'
                    wr.writerow(data_dict)
            self.unanswered = True
            
    def get_arguments(self):
        xml_kbase = XMLKnowledgeBase(self.object_xml_filename, self.location_xml_filename)
        qa_bot = QuestionAnswerer(self.template_filename, xml_kbase)
        answer = qa_bot.answer_question(self.speech_transcript)
        self.unanswered = False
        if type(answer) == tuple:
            return answer
        return ("", "")
    
    def answer(self, speech):
        
        xml_kbase = XMLKnowledgeBase(self.object_xml_filename, self.location_xml_filename)
        qa_bot = QuestionAnswerer(self.template_filename, xml_kbase)
        if self.speech_transcript == None and self.question_count > 5 and self.can_clarify == True:
            answerstr = "Can you repeat the question?"
            self.can_clarify = False
        elif self.speech_transcript == None:
            answerstr = "I don't know"
            self.can_clarify = True
        else:
            answerstr = qa_bot.answer_question(self.speech_transcript)
            self.can_clarify = True
        print("Unanswered: %s", str(self.unanswered))
        if self.unanswered:
            self.speech_time = rospy.get_time()
            speech.say(answerstr, wait=True)
            self.speech_time = rospy.get_time()
            self.unanswered = False
        
    def is_unanswered(self):
        return self.unanswered
        
    def get_question_count(self):
        return self.question_count
        
    def get_listen_time(self):
        return self.listen_time
       
    def set_speech_time(self, setting):
        self.speech_time = setting
       
        
    
    

