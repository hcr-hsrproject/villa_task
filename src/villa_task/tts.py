"""
Text to Speech Handler
"""

import rospy
from tmc_msgs.msg import Voice
from std_msgs.msg import String
import time

class TextToSpeech(object):

    ENGLISH = Voice.kEnglish


    def __init__(self, timeout=3.0):
        self.tts = rospy.Publisher('/talk_request', Voice, queue_size=1)

        # Block until tts connection is achieved. Speech will not work without this
        result = TextToSpeech.wait_until(lambda: self.tts.get_num_connections() > 0, timeout=timeout, polling=0.1)
        if not result:
            rospy.logerr("Failed to connect to /talk_request")

        # Keep in the loop of what's being said
        self.talking_sentence = ''
        rospy.Subscriber('/talking_sentence', String, self.__talking_sentence)


    """
    Speak a given sentence
    wait: Wait for the sentence to complete
    """
    def say(self, sentence, wait=False):
        self.tts.publish(Voice(sentence=sentence, language=TextToSpeech.ENGLISH))
        if wait:
            # Wait until our sentence comes up
            TextToSpeech.wait_until(lambda: self.talking_sentence == sentence, polling=0.1)
            # Wait for silence
            TextToSpeech.wait_until(lambda: not self.talking_sentence, polling=0.1)


    """
    Blocks until condition is met or times out
    """
    @staticmethod
    def wait_until(func, timeout=None, polling=1.0, *args, **kwargs):
        if timeout is not None:
            end_time = time.time() + timeout
        else:
            end_time = time.time()
        while (time.time() < end_time or timeout is None):
            cond = func(*args, **kwargs)
            if cond:
                return True
            time.sleep(polling)
        return False


    def __talking_sentence(self, sentence):
        self.talking_sentence = sentence.data
