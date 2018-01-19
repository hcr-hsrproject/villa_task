import rospy
from tf2_py._tf2 import ExtrapolationException
from villa_manipulation.srv import ObjectFrameData, ObjectFrameDataRequest


class TransformManager:
    def __init__(self, whole_body):
        self.whole_body = whole_body
        self.object_frames_client = rospy.ServiceProxy("broadcast_object_frames", ObjectFrameData)
        self.object_frames_client.wait_for_service(5)
        self._relations = []

    def get_transform(self, target, source):
        rospy.loginfo("Waiting for transform from {} to {}".format(source, target))
        transform = None
        while not rospy.is_shutdown():
            try:
                transform = self.whole_body._tf2_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(5))
                break
            except ExtrapolationException as e:
                continue
        return transform

    def add_relation(self, relation):
        self.remove_relation(relation)
        self._relations.append(relation)

    def add_relations(self, relations):
        for relation in relations:
            self.add_relation(relation)

    def remove_relation(self, relation):
        parent = relation[0]
        child = relation[1]
        for existing in self._relations:
            if existing[0] == parent and existing[1] == child:
                self._relations.remove(existing)
                break

    def remove_relation_by_object_name(self, object_name):
        for parent, child, pose in self._relations:
            if object_name == child:
                self._relations.remove((parent, child, pose))

    def broadcast_links(self):
        try:
            rospy.loginfo("Updating object frames")
            tf_frames_request = ObjectFrameDataRequest()
            parent_frames = [relation[0] for relation in self._relations]
            child_frames = [relation[1] for relation in self._relations]
            poses = [relation[2] for relation in self._relations]
            tf_frames_request.parent_frames = parent_frames
            tf_frames_request.child_frames = child_frames
            tf_frames_request.poses = poses
            tf_res = self.object_frames_client(tf_frames_request)
            if tf_res is False:
                rospy.logerr("Update failed")
        except rospy.ServiceException as e:
            rospy.logerr(e)
