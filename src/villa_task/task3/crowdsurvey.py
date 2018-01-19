#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class ScanCrowd():
    def __init__(self):
        self.genders =  []
        print('Subscribing to gender detection..')
        gender_detection = rospy.Subscriber('/villa/gender_identification', String, self.__update_genders)
    
    def __update_genders(self, g):
        if len(self.genders) < 11:
            self.genders.append(g.data.split(','))
        else:
            del self.genders[0]
        self.genders.append(g.data.split(','))
    
    def get_gender(self):
        m_count = 0
        f_count = 0
        for g in self.genders:
            if g[0] == 'M':
                m_count = m_count + 1
            else:
                f_count = f_count + 1
        if(m_count > f_count):
            return 'M'
        else:
            return 'F'
        
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