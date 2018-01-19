import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from villa_helpers.srv import StartCountdown
from std_msgs.msg import Int32

pose_publisher = rospy.Publisher("pose_visulizer", PoseStamped, queue_size=10)
object_labels_pub = rospy.Publisher("objectlabels", MarkerArray, queue_size=10)

timer_visualization_service = rospy.ServiceProxy("start_countdown_timer", StartCountdown)

def start_task_timer(start_pose):
    timer_position = PointStamped()
    timer_position.header.frame_id = "/map"
    timer_position.header.stamp = rospy.Time().now()
    timer_position.point.x = start_pose[0]
    timer_position.point.y = start_pose[1]
    timer_position.point.z = 1.5
    timer_visualization_service(Int32(240), timer_position)

def construct_rviz_marker( label, obj_id, x, y, z):
    marker = Marker()
    marker.header.frame_id = '/table'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.ns = 'labels'
    marker.id = obj_id
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.
    marker.lifetime = rospy.Duration(0)
    marker.scale.z = 0.06
    marker.text = label
    marker.color.r = 0.
    marker.color.g = 0.
    marker.color.b = 0.
    marker.color.a = 1.
    return marker
