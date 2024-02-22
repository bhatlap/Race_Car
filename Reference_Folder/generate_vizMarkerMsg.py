
# ROS Imports
import rospy
from visualization_msgs.msg import Marker


def generate_vizMarkerMsg(trajMsg, id, namespace, colors=None):
    """
    Function to create a marker message to visualize a pre-computed trajectory

    :input:
        trajMsg : Trajectory message of type nav_msgs/Path

    :Output:
        markerMsg : Marker message of type visualization_msgs/Marker
    """
    if colors is None:
        colors = [0.25, 0.0, 0.75]
    markerMsg = Marker()
    point = []
    # Pose marker message
    markerMsg.header.frame_id = trajMsg.header.frame_id
    markerMsg.header.stamp = rospy.Time.now()
    markerMsg.id = id
    markerMsg.ns = namespace
    markerMsg.type = Marker.SPHERE_LIST
    markerMsg.action = Marker.ADD
    markerMsg.pose.orientation.w = 1.0
    markerMsg.scale.x = 0.1
    markerMsg.scale.y = 0.1
    markerMsg.scale.z = 0.1
    markerMsg.color.r = colors[0]
    markerMsg.color.g = colors[1]
    markerMsg.color.b = colors[2]

    markerMsg.color.a = 1.0

    pts = trajMsg.poses
    for pt in pts:
        point.append(pt.pose.position)

    markerMsg.points = point
    return markerMsg
