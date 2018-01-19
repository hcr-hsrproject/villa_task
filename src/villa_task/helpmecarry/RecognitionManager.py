import rospy
from face_recognition_ros.srv import take_photo, face_recognition_srv


class RecognitionManager:
    def __init__(self):
        self.face_recognition_client = rospy.ServiceProxy("face_recognition_run", face_recognition_srv)
        self.face_recognition_client.wait_for_service(timeout=5)
        # self.take_photo_client = rospy.ServiceProxy("take_photo_srv", take_photo)
        # self.take_photo_client.wait_for_service(timeout=5)

    def capture_operator_photo(self):
        print("I am calling")
        # take_photo_result = self.take_photo_client(True)
        #print take_photo_result.saved_name

