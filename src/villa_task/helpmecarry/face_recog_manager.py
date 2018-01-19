#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from os import listdir
from os.path import isfile, join, exists
import numpy as np
import std_msgs.msg
# FIXME: This import seems to load a database and cause a segfault
import face_recognition

from rude_carnie.detect import face_detection_model



class FaceRecogManager(object):
    def __init__(self, wait=0.0):
        image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        rospy.Subscriber(image_topic, Image, self.image_callback)
        self.string_pub = rospy.Publisher('/face_detected_name', String, queue_size=10)
        self.bridge = CvBridge()
        self.path = exists("known_faces")
        print self.path
        self.takepicture = False
        self.threshold = 0.6
        self.known_folder = "/home/mk/robocup_home_ws/src/villa_perception/face_recognition/scripts/known_faces"

        self.known_imgs = [f for f in listdir(self.known_folder) if isfile(join(self.known_folder, f))]
        print self.known_imgs
        self.known_img_names = []
        self.known_encodings = []
        self.count = 0
        self.sentence = sentence = " q "

        for img_name in self.known_imgs:
            img = face_recognition.load_image_file(self.known_folder + '/' + img_name)
            print img_name
            self.known_encodings.append(face_recognition.face_encodings(img)[0])
            dot_idx = img_name.rfind('.')
            name = img_name[0:dot_idx]
            self.known_img_names.append(name)

        # self.listener()

    def listener(self, wait=0.0):
        print("Subscribing to Images for face_recognition")
        rospy.spin()

    def key_callback(self, msg):  # "p"
        self.code = msg.code
        if self.code == 112:
            self.takepicture = True
            print('take picture')
        print msg.code

    def crop_face(self, image_data):
        image_batch = image_data
        files = []
        face_detect = face_detection_model('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')
        face_files = face_detect.run(image_data)
        # print(face_files)
        return face_files

    def image_callback(self, msg):
        # print("Received an image!")
        self.detected_msg = std_msgs.msg.String()
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            facelists = self.crop_face(cv2_img)

            for face in facelists:
                unknown_encoding = face_recognition.face_encodings(face)[0]
                distances = face_recognition.face_distance(self.known_encodings, unknown_encoding)
                if (min(distances) < self.threshold):
                    match_idx = np.argmin(distances)
                    print('matched: ' + self.known_img_names[match_idx])
                    self.detected_msg.data = self.known_img_names[match_idx]
                    self.count = self.count + 1
                    if (self.count > 5):
                        self.sentence = " I can see " + self.known_img_names[match_idx]
                        self.string_pub.publish(self.detected_msg)
                        self.count = 0
                    # self.tts.say('matched')

                else:
                    print('unknown')
                    self.count = 0

        except CvBridgeError, e:
            print(e)

        if self.takepicture:
            cv2.imwrite('new_image.jpeg', cv2_img)
            print('save picture')
            # self.detected_msg.data="Take a photo"
            # self.string_pub.publish(self.detected_msg)
            self.takepicture = False
        # else:
        # Save your OpenCV2 image as a jpeg
        # cv2.imwrite('minkyu_image.jpeg', cv2_img)
