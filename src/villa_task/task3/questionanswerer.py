#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from spr_qa.xml_kbase import XMLKnowledgeBase
from spr_qa.spr_qa_main import QuestionAnswerer
from spr_qa.correct_utterance import CorrectUtterance

class AnswerBase():
    def __init__(self):
        self.listen_time = 0
        self.speech_transcript = ''
        self.question_count = 0
        self.unanswered = False
        self.speech_time = 0
        self.time_needed = True
        self.template_filename = "/home/hsr-user/workspaces/sound/src/villa_natural_language/spr_qa/src/spr_qa/resources/spr_qa_updated.csv"
        self.location_xml_filename = "/home/hsr-user/workspaces/sound/src/villa_natural_language/resources/Locations_mod.xml"
        self.object_xml_filename = "/home/hsr-user/workspaces/sound/src/villa_natural_language/resources/Objects_mod.xml"
        self.corrector = CorrectUtterance()
        
        print('Subscribing to speech detection..')
        speech_detection = rospy.Subscriber('/villa/speech_transcripts', String, self.__update_speech)
    
    def __update_speech(self, s):
        if rospy.get_time() - self.speech_time > 4 and self.time_needed:
            self.listen_time = rospy.get_time()
            self.speech_transcript = self.corrector.correct(s.data)
            f = open('/home/hsr-user/workspaces/sound/src/villa_task/scripts/save_sentences.txt', 'a')
            print("--- QUESTION COUNT: %d ---" % self.question_count)
            print("--- HEARD: %s ---" % s.data)
            if self.speech_transcript == None and self.question_count >= 5:
                print("--- NO CORRECTION FOUND ---")
            elif self.speech_transcript == None:
                self.question_count = self.question_count + 1
                print("--- NO CORRECTION FOUND ---")
            else:
                self.question_count = self.question_count + 1
                print("--- CORRECTED: %s ---" % self.speech_transcript)
            f.write(s.data + '\n')
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
        if self.speech_transcript == None and self.question_count > 5:
            answerstr = "Can you repeat the question?"
        elif self.speech_transcript == None:
            answerstr = "I don't know"
        else:
            answerstr = qa_bot.answer_question(self.speech_transcript)
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
        
    
    
