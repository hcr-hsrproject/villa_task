from sensor_msgs.msg import PointCloud2
import rospy


class OctomapManager:
    def __init__(self):
        rospy.Subscriber('octomap_point_cloud_centers', PointCloud2, self._octomap_cb)

        self.ready_to_process_octomap = False
        self.octomap_cloud = None

    def get_octomap_cloud(self):
        self.octomap_cloud = None
        self.ready_to_process_octomap = True
        self.wait_for_data()

    def _octomap_cb(self, data):
        if not self.ready_to_process_octomap:
            return
        if self.octomap_cloud is None:
            self.octomap_cloud = data

    def wait_for_data(self):
        while self.octomap_cloud is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "Waiting for octomap cloud")
            continue
