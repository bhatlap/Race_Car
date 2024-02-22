
# ROS Imports
import rospy
from visualization_msgs.msg import Marker


def generate_point_marker_msg(trajMsg):
    """
    Function to create a marker message to visualize a pre-computed trajectory

    :input:
        trajMsg : Trajectory message of type nav_msgs/Path

    :Output:
        markerMsg : Marker message of type visualization_msgs/Marker
    """
    markerMsg = Marker()

    # Pose marker message
    markerMsg.header.frame_id = trajMsg.header.frame_id
    markerMsg.header.stamp = rospy.Time.now()
    markerMsg.id = 1
    markerMsg.ns = 'Reference_trajectory_points'
    markerMsg.type = Marker.POINTS
    markerMsg.action = Marker.ADD
    markerMsg.pose.orientation.w = 1.00
    markerMsg.scale.x = 0.05
    markerMsg.scale.y = 0.05
    markerMsg.scale.z = 0.05
    markerMsg.color.r = 0.25
    markerMsg.color.b = 0.75
    markerMsg.color.a = 1.0

    pts = trajMsg.poses
    for pt in pts:
        point = pt.pose.position
        markerMsg.points.append(point)

    return markerMsg
