
# ROS Imports
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler


def generate_trajectoryMarkerMsg(trajMsg, id, namespace, colors=None):
    """
    Function to create a marker message to visualize a pre-computed trajectory

    :input:
        trajMsg : Trajectory message of type nav_msgs/Path

    :Output:
        markerMsg : Marker message of type visualization_msgs/Marker
    """
    if colors is None:
        colors = [0.25, 0.0, 0.75]
    trajmarkerMsg = Marker()
    point = []
    # Pose marker message
    trajmarkerMsg.header.frame_id = trajMsg.header.frame_id
    trajmarkerMsg.header.stamp = rospy.Time.now()
    trajmarkerMsg.id = id
    trajmarkerMsg.ns = namespace
    trajmarkerMsg.type = Marker.SPHERE_LIST
    trajmarkerMsg.action = Marker.ADD
    trajmarkerMsg.pose.orientation.w = 1.0
    trajmarkerMsg.scale.x = 0.1
    trajmarkerMsg.scale.y = 0.1
    trajmarkerMsg.scale.z = 0.1
    trajmarkerMsg.color.r = colors[0]
    trajmarkerMsg.color.g = colors[1]
    trajmarkerMsg.color.b = colors[2]

    trajmarkerMsg.color.a = 1.0

    pts = trajMsg.poses
    for pt in pts:
        point.append(pt.pose.position)

    trajmarkerMsg.points = point
    return trajmarkerMsg


def generate_LineSegment(coordinates, id, namespace='Line_Segment', colors=None):

    if colors is None:
        colors = [0.0, 1.0, 0.0]

    point = []
    lineMsg = Marker()
    lineMsg.header.frame_id = 'map'
    lineMsg.id = id
    lineMsg.ns = namespace
    lineMsg.type = Marker.SPHERE_LIST
    lineMsg.action = Marker.ADD
    lineMsg.pose.orientation.w = 1.0
    lineMsg.scale.x = 0.1
    lineMsg.scale.y = 0.1
    lineMsg.scale.z = 0.1
    lineMsg.color.a = 1.0
    lineMsg.color.r = colors[0]
    lineMsg.color.g = colors[1]
    lineMsg.color.b = colors[2]

    for x,y in coordinates:
        point.append(Point(x,y,0.0))

    lineMsg.points = point
    return lineMsg


def generate_PointMarkerMsg(x, y, namespace='point', id=3, colors=None):

    if colors is None:
        colors = [0.0, 1.0, 0.0]

    ptMarkerMsg = Marker()

    ptMarkerMsg.header.frame_id = 'map'
    ptMarkerMsg.header.stamp = rospy.Time.now()
    ptMarkerMsg.id = id
    ptMarkerMsg.ns = namespace
    ptMarkerMsg.type = Marker.SPHERE
    ptMarkerMsg.action = Marker.ADD
    ptMarkerMsg.pose.position = Point(x, y, 0.0)
    ptMarkerMsg.pose.orientation.w = 1.0
    ptMarkerMsg.scale.x = 0.1
    ptMarkerMsg.scale.y = 0.1
    ptMarkerMsg.scale.z = 0.1
    ptMarkerMsg.color.r = colors[0]
    ptMarkerMsg.color.g = colors[1]
    ptMarkerMsg.color.b = colors[2]
    ptMarkerMsg.color.a = 1.0

    return ptMarkerMsg


def generate_arrow_msg(pose, id, namespace='Arrow',colors=None):
    if colors is None:
        colors = [0.0, 1.0, 1.0]

    arrowMsg = Marker()

    x = pose[0]
    y = pose[1]
    psi = pose[2]

    q = quaternion_from_euler(0,0,psi)

    arrowMsg.header.frame_id = 'map'
    arrowMsg.header.stamp = rospy.Time.now()
    arrowMsg.id = id
    arrowMsg.ns = namespace
    arrowMsg.type = Marker.ARROW
    arrowMsg.action = Marker.ADD
    arrowMsg.scale.x = 0.5
    arrowMsg.scale.y = 0.1
    arrowMsg.scale.z = 0.1
    arrowMsg.color.a = 1.0
    arrowMsg.color.r = colors[0]
    arrowMsg.color.g = colors[1]
    arrowMsg.color.b = colors[2]
    arrowMsg.pose.position = Point(x, y, 0.0)
    arrowMsg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    return arrowMsg



