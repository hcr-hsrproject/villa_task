#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import xml.etree.cElementTree as ET
import os
import rospkg

class ScanCrowd():

    def __init__(self):
        self.rp = rospkg.RosPack()
        self.save_data = os.path.join(self.rp.get_path("spr_qa"),"src/spr_qa/resources/xml_files/people_info.xml") 
        self.genders =  []
        self.ages = []
        rospy.loginfo('Subscribing to gender detection..')
        gender_detection = rospy.Subscriber('/villa/gender_identification', String, self.__update_genders)
        rospy.loginfo('Subscribing to age detection..')
        age_detection = rospy.Subscriber('/villa/age_identification', String, self.__update_ages)

        self.save_genders = []
        self.save_ages = []


    def count_genders(self, speech):
        #wait for the robot to see people
        while len(self.genders) == 0:
            rospy.sleep(0.5) # Give gender processing some time
        rospy.sleep(0.5)
        # Count the size of the crowd (male/female)
        g = self.genders[-1]
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


    def get_genders(self):
        return self.save_genders[-1]


    def get_ages(self):
        return self.save_ages[-1]


    def save_information(self):
        self.save_genders = self.genders
        self.save_ages = self.ages
        root = ET.Element("people")
        length = min(len(self.save_genders[-1]), len(self.save_ages[-1]))
        for i in range(length):
            ET.SubElement(root, "person", gender = self.save_genders[-1][i], age = self.save_ages[-1][i]).text = str(i)
        tree = ET.ElementTree(root)
        tree.write(self.save_data)


    def __update_genders(self, g):
        if len(self.genders) < 11:
            self.genders.append(g.data.split(','))
        else:
            del self.genders[0]
        self.genders.append(g.data.split(','))


    def __update_ages(self, a):
        if len(self.ages) < 11:
            self.ages.append(a.data.split(','))
        else:
            del self.ages[0]
        self.ages.append(a.data.split(','))

