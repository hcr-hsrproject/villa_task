import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from villa_helpers.srv import StartCountdown
from std_msgs.msg import Int32
from hsrb_interface import geometry

pose_publisher = rospy.Publisher("visualization/pose", PoseStamped, queue_size=10, latch=True)

object_labels_pub = rospy.Publisher("visualization/object_labels", MarkerArray, queue_size=10, latch=True)

timer_visualization_service = rospy.ServiceProxy("visualization/start_countdown_timer", StartCountdown)

def start_task_timer(start_pose):
    timer_position = PointStamped()
    timer_position.header.frame_id = "/map"
    timer_position.header.stamp = rospy.Time().now()
    timer_position.point.x = start_pose[0]
    timer_position.point.y = start_pose[1]
    timer_position.point.z = 1.5
    timer_visualization_service(Int32(240), timer_position)

def publish_goal_pose(pose, frame_id):
    pose_stamped = PoseStamped()
    pose_stamped.pose = geometry.tuples_to_pose(pose)
    pose_stamped.header.frame_id = frame_id
    pose_publisher.publish(pose_stamped)

def construct_rviz_marker(label, obj_id, x, y, z):
    marker = Marker()
    marker.header.frame_id = '/table'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.ns = 'labels'
    marker.id = obj_id
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z + 0.08
    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.
    marker.lifetime = rospy.Duration(0)
    marker.scale.z = 0.08
    marker.text = label
    marker.color.r = 0.
    marker.color.g = 0.
    marker.color.b = 0.
    marker.color.a = 1.
    return marker
