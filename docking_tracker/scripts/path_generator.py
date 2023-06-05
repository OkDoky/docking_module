#!/usr/bin/python3
import rospy
from math import atan2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import tf2_ros
from std_srvs.srv import Empty, EmptyResponse

def interpolate_points(start_x, start_y, end_x, end_y, increment):
    num_points = int(max(abs(end_x-start_x), abs(end_y-start_y))/increment)
    x = start_x
    y = start_y
    incre_x = (end_x - start_x)/num_points
    incre_y = (end_y - start_y)/num_points
    points = []
    points.append((x,y))
    for i in range(num_points):
        x += incre_x
        y += incre_y
        points.append((x,y))
    return points

def event_callback(req):
    create_path()
    res = EmptyResponse()
    return res

def create_path():
    global dst_x, dst_y
    dst_x = rospy.get_param("~dst_x", 2.0)
    dst_y = rospy.get_param("~dst_y", 0.0)
    # Get Current pose from tfBuffer
    _trans = tfBuffer.lookup_transform("odom", "base_footprint",rospy.Time(),timeout=rospy.Duration(10.0)).transform
    cur_x = _trans.translation.x
    cur_y = _trans.translation.y
    rospy.logdebug("[PathGenerator] dst_x : %s"%dst_x)
    rospy.logdebug("[PathGenerator] dst_y : %s"%dst_y)
    rospy.logdebug("[PathGenerator] cur_x : %s"%cur_x)
    rospy.logdebug("[PathGenerator] cur_y : %s"%cur_y)
    rospy.logdebug("[PathGenerator] increment : %s"%increment)
    
    # Calculate the intermediate points
    intermediate_points = interpolate_points(cur_x, cur_y, dst_x, dst_y, increment)
    
    # init direction
    dx = dst_x - cur_x
    dy = dst_y - cur_y
    theta = atan2(dy, dx)
    quat = quaternion_from_euler(0,0,theta)
    
    
    # Create the path message
    path_msg = Path()
    path_msg.header.frame_id = 'odom'
    path_msg.header.stamp = rospy.Time.now()
    # Create a pose for the destination
    for point in intermediate_points:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom"
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        # Add the destination pose to the path
        path_msg.poses.append(pose)
    
    # Publish the path
    path_pub.publish(path_msg)
    print("publish path finish")

# Example usage
if __name__ == '__main__':
    rospy.init_node('path_generator')
    path_pub = rospy.Publisher('/desired_path', Path, queue_size=10)
    event_server = rospy.Service("~setPlan", Empty, event_callback)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    # SetParams
    global dst_x, dst_y
    dst_x = rospy.get_param("~dst_x", 2.0)
    dst_y = rospy.get_param("~dst_y", 0.0)
    increment = rospy.get_param("~increment", 0.05)

    rospy.spin()
