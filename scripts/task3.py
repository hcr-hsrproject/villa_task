#!/usr/bin/env python

"""
Task 3 as specified in 5.3.3 of the Rulebook
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
from villa_helpers.fast_move import FastMove
import re
from hsrb_interface import geometry
from spr_qa.xml_kbase import XMLKnowledgeBase
from spr_qa.spr_qa_main import QuestionAnswerer

genders = []
ages = []
speech_time = 0
speech_transcript = ''
listen = None
question_count = 0
speech = None
unanswered = False
speak_time = 0
turning = False

def update_genders(g):
    global genders
    if len(genders) < 11:
        genders.append(g.data.split(','))
    else:
        del genders[0]
    genders.append(g.data.split(','))
    
    #genders = g.data.split(',')

def update_ages(a):
    global ages
    if len(ages) < 11:
        ages.append(a.data.split(','))
    else:
        del ages[0]
    ages.append(a.data.split(','))
    
    #ages = a.data.split(',')

def get_gender():
    global genders
    print("Genders:",genders)
    m_count = 0
    f_count = 0
    for g in genders:
        if g[0] == 'M':
            m_count = m_count + 1
        else:
            f_count = f_count + 1
    if(m_count > f_count):
        return 'M'
    else:
        return 'F'

def get_age():
    global ages
    print("Ages:",ages)
    counts = [0,0,0,0,0,0,0,0]
    ranges = ['0-2','4-6','8-12','15-20','25-32','38-43','48-53','60-100']
    for a in ages:
        for r in range(len(ranges)):
            if a[0] == ranges[r]:
                counts[r] = counts[r] + 1
    max = 0
    for c in range(len(counts)):
        if counts[c] > counts[max]:
            max = c
    return ranges[max]
 


def update_speech(s):
    global speech_time
    global speech_transcript
    global question_count
    global listen
    global speech
    global unanswered
    global speak_time
    print("Time difference:", rospy.get_time() - speak_time)
    if rospy.get_time() - speak_time > 3:
        speech_time = rospy.get_time()
        speech_transcript = s.data
        f = open('/home/hsr-user/workspaces/sound/src/villa_task/scripts/save_sentences.txt', 'a')
        f.write(speech_transcript + '\n')
        question_count = question_count + 1
        print("--- QUESTION COUNT: %d ---" % question_count)
        print("--- SPEECH: %s ---" % s.data)
        unanswered = True
        #if question_count <= 5:
        #    answer(speech)

def robot_moved(fastmove, noise_stop, whole_body, speech, motionhandler):
    #noise_stop.publish(True)
    global listen
    global unanswered
    global turning
    motionhandler.stop()
    detect_person = LocatePerson(fastmove)
    rospy.sleep(1.5)
    if(detect_person.found_person):
        print("I found a person at angle:",detect_person.current_angle)
        fastmove.go(0,0,detect_person.current_angle, 100, relative=True)
    #speech.say(random.choice(responses), wait=True)
    print("Person at:",detect_person.x, detect_person.y,detect_person.w,detect_person.h)
    answer(speech)
    turning = False
    print("Restarting motion handler")
    motionhandler.start()

def begin_move(noise_start, motionhandler):
    #noise_start.publish(True)
    global question_count
    global turning
    motionhandler.stop()
    rospy.sleep(2)
    motionhandler.start()
    # If time is less than 5 then move
    if abs(rospy.get_time() - speech_time) < 5 and question_count > 5:
        print("ALLOWING MOVE")
        turning = True
        return True
    else:
        if(question_count <= 5):
            print("STOPPED DUE TO RIDDLE GAME")
        else:
            print("STOPPED DUE TO SPEECH")
        return False
        
def answer(speech):
    global unanswered
    global speak_time
    template_filename = "/home/hsr-user/workspaces/sound/src/villa_natural_language/spr_qa/src/spr_qa/resources/spr_question_answer_templates.csv"
    location_xml_filename = "/home/hsr-user/workspaces/sound/src/villa_natural_language/resources/Locations_mod.xml"
    object_xml_filename = "/home/hsr-user/workspaces/sound/src/villa_natural_language/resources/Objects_mod.xml"
    
    xml_kbase = XMLKnowledgeBase(object_xml_filename, location_xml_filename)
    qa_bot = QuestionAnswerer(template_filename, xml_kbase)
    answerstr = qa_bot.answer_question(speech_transcript)
    speech.say(answerstr, wait=True)
    speak_time = rospy.get_time()
    unanswered = False
    rospy.sleep(4)
        

def act(robot):
    global listen
    global speech
    global unanswered
    global speech_time
    global question_count
    global turning
    global speak_time
    print('Subscribing to whole body...')
    whole_body = robot.get('whole_body')
    print('Subscribing to omni base...')
    omni_base = robot.get('omni_base')
    fastmove = FastMove(omni_base)
    print('Subscribing to tts...')
    speech = tts.TextToSpeech()
    print('Subscribing to gender detection..')
    gender_detection = rospy.Subscriber('/villa/gender_identification', String, update_genders)
    print('Subscribing to age detection..')
    age_detection = rospy.Subscriber('/villa/age_identification', String, update_ages)
    noise_start = rospy.Publisher('/villa/sound_localization/noise/start', Bool, queue_size=1)
    noise_stop = rospy.Publisher('/villa/sound_localization/noise/stop', Bool, queue_size=1)
    motionhandler = MotionHandler(fastmove, start_motion_callback=functools.partial(begin_move, noise_start), end_motion_callback=functools.partial(robot_moved, fastmove, noise_stop, whole_body, speech))

    whole_body.move_to_neutral()
    
    # Start by having the robot say it wants to play
    speech.say("Let's play a riddle game", wait=True)
    speech_time = rospy.get_time()

    # Wait for people to get in position and then turn
    # Calibrate noise while we're waiting
    #noise_start.publish(True)
    rospy.sleep(10)
    #noise_stop.publish(True)

    # Turn 180
    print("Waiting to stop head...")
    rospy.wait_for_service('/viewpoint_controller/stop')
    stop_head = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_head()
    print("Stopped head. Turning.")
    
    fastmove.go(0., 0., math.pi, 100., relative=True)
    
    # Reset position
    whole_body.move_to_neutral()

    
    while len(genders) == 0:
        rospy.sleep(0.5) # Give gender processing some time
    rospy.sleep(0.5)
    # Count the size of the crowd (male/female)
    g = genders[-1]
    crowd_count = len(g)
    if crowd_count == 1:
        sizestr = "There is %d person." % (crowd_count)
    else:
        sizestr = "There are %d people." % (crowd_count)
    speech.say(sizestr, wait = True)
    rospy.sleep(0.5) #natural sounding pause
    male_count = g.count('M')
    female_count = g.count('F')
    if male_count == 1 and female_count == 1:
        genderstr = "There is %d man and %d woman." % (male_count,female_count)
    elif male_count == 1:
        genderstr = "There is %d man and %d women." % (male_count,female_count)
    elif female_count == 1:
        genderstr = "There are %d men and %d woman." % (male_count, female_count)
    else:
        genderstr = "There are %d men and %d women." % (male_count, female_count)
    speech.say(genderstr, wait=True)
    speech_time = rospy.get_time()
    rospy.sleep(0.5) # natural sounding pause
    speech.say("Who wants to play riddles with me?", wait=True)
    speech_time = rospy.get_time()
    rospy.sleep(0.5)
    print('Subscribing to speech detection..')
    speech_detection = rospy.Subscriber('/villa/speech_transcripts', String, update_speech)

    # Start listening to localization
    print("Starting motion handler")
    motionhandler.start()

    while not rospy.is_shutdown():
        if unanswered == True and ((rospy.get_time() - speech_time > 8 and not turning) or question_count <= 5):
            print("Answering because HARK took too long")
            answer(speech)


with Robot() as robot:
    act(robot)

