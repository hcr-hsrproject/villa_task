import rospy
from tf2_py._tf2 import ExtrapolationException
from villa_manipulation.srv import object_frame_data, object_frame_dataRequest
from villa_task.task5.speak import robot_say

class TransformManager:
    def __init__(self, whole_body):
        self.whole_body = whole_body
        self.object_frames_client = rospy.ServiceProxy("object_frames", object_frame_data)
        self.object_frames_client.wait_for_service(10)
        self._relations = []

    def get_transform(self, target, source):
        print "Waiting for transform from ", source, "to ", target, "..."
        transform = None
        while not rospy.is_shutdown():
            try:
                transform = self.whole_body._tf2_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(1.0))
                break
            except ExtrapolationException as e:
                continue
        return transform

    def add_relation(self, relation):
        parent = relation[0]
        child = relation[1]
        for existing in self._relations:
            if existing[0] == parent and existing[1] == child:
                self._relations.remove(existing)
                break
        self._relations.append(relation)

    def add_relations(self, relations):
        for relation in relations:
            self.add_relation(relation)

    def broadcast_links(self):
        try:
            robot_say("Broadcasting links")
            tf_frames_request = object_frame_dataRequest()
            parent_frames = [relation[0] for relation in self._relations]
            child_frames = [relation[1] for relation in self._relations]
            poses = [relation[2] for relation in self._relations]
            tf_frames_request.parent_frames = parent_frames
            tf_frames_request.child_frames = child_frames
            tf_frames_request.poses = poses
            tf_res = self.object_frames_client(tf_frames_request)
            print "tf response: ", tf_res.success
        except rospy.ServiceException as e:
            rospy.logerr(e)
