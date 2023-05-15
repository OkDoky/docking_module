#!/usr/bin/env python3

import rospy
import os
import sys
import tf
import numpy as np
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, QuaternionStamped, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix
sys.path.append(os.path.dirname(__file__))
print(os.path.dirname(__file__))

from planner import quintic_polynomials_planner

class tf_Transform:
    def __init__(self):
        _aruco = TFMessage()

Aurco_tf = TFMessage()
Aruco_pose = Quaternion()
Odom_pose = Quaternion()
robot_odom = Odometry()
def compute_plan(start_point, start_yaw, end_point, end_yaw, straight_end, dt=0.1):
    start_ang = np.deg2rad(start_yaw)
    end_ang = np.deg2rad(end_yaw)
    time, x, y, yaw, v, a, j = quintic_polynomials_planner(\
        start_point.x, start_point.y, start_ang, end_point.x, end_point.y, end_ang, dt)
    path = Path()

    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "odom"

    for i, t in enumerate(time):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = path.header.stamp + rospy.Duration.from_sec(t)
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = x[i]
        pose_stamped.pose.position.y = y[i]
        pose_stamped.pose.position.z = 0
        quaternion = Quaternion()
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = 0
        quaternion.w = 1
        pose_stamped.pose.orientation = quaternion
        path.poses.append(pose_stamped)   
    line_start = np.array([end_point.x, end_point.y, 0])
    line_end = np.array([straight_end.x, straight_end.y, 0])
    dist = int(np.linalg.norm(line_end - line_start)*100 + 1)
    points = np.linspace(line_start, line_end, dist)

    for point in points:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = 0
        quaternion = Quaternion()
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = 0
        quaternion.w = 1
        pose_stamped.pose.orientation = quaternion
        path.poses.append(pose_stamped)
    return path

def callback_arucoTF(tf_data):
    global Aurco_tf
    Aurco_tf = tf_data


def callback_odomTF(sub_odom):
    global robot_odom
    robot_odom = sub_odom

def cal_plan(displacement):
    global Aurco_tf, robot_odom
    if len(Aurco_tf.transforms) != 0 and robot_odom.header.frame_id != "":
        tf_trans = Aurco_tf.transforms[0].transform.translation
        point_origin = Point(x=0., y=0., z=0.)
        point = Point(x=0., y=0., z=displacement)

        origin_quaternion = Aurco_tf.transforms[0].transform.rotation
        origin_translation = (tf_trans.x, tf_trans.y, tf_trans.z)
        origin_matrix = np.dot(tf.transformations.translation_matrix(origin_translation), tf.transformations.quaternion_matrix\
                        ([origin_quaternion.x, origin_quaternion.y, origin_quaternion.z, origin_quaternion.w]))
        
        origin_point = np.array([point_origin.x, point_origin.y, point_origin.z, 1]) 
        origin_new = np.dot(origin_matrix, origin_point) # marker origin point(base odom)
        origin_pos = Point(x=origin_new[0], y=origin_new[1], z=origin_new[2]) # marker origin point(base odom)

        v = np.array([point.x, point.y, point.z, 1]) # offset point
        v_new = np.dot(origin_matrix,v) # offset point
        point_new = Point(x=v_new[0], y=v_new[1], z=v_new[2]) # offset point

        start_point = Point(x=robot_odom.pose.pose.position.x, y=robot_odom.pose.pose.position.y, z=0.) # now robot odom position
        print("start_point")
        print(start_point)
        print("point_new")
        print(point_new)
        print("origin_pose")
        print(origin_pos)
        print("------------------")
        return compute_plan(start_point, 0, point_new, 0, origin_pos)
    else: return Path()

def sub_TF():
    rospy.init_node('subscribeTF', anonymous=True)
    displacement = float(rospy.get_param("displacement", "0.5"))
    print(displacement)
    rospy.Subscriber("tf_list", TFMessage, callback_arucoTF)
    rospy.Subscriber("odom", Odometry, callback_odomTF)
    pub = rospy.Publisher('compute_Path', Path, queue_size=10)
    while not rospy.is_shutdown():
        plan = cal_plan(displacement)
        if len(plan.poses) != 0:
            pub.publish(plan)
        rospy.sleep(0.1)
    

if __name__=='__main__':
    sub_TF()