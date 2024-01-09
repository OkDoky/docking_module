#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import tf
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, QuaternionStamped, Point, Quaternion, PoseStamped, TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix
sys.path.append(os.path.dirname(__file__))
print(os.path.dirname(__file__))

from planner import quintic_polynomials_planner

# Aurco_tf = TFMessage()
Aruco_tf = TransformStamped()
Aruco_pose = Quaternion()
Odom_pose = Quaternion()
robot_odom = Odometry()
def compute_plan(start_point, start_yaw, end_point, end_yaw, 
                 straight_end, frame, sv, ev, dt=0.1):
    global last_plan_info
    
    start_ang = np.deg2rad(start_yaw)
    end_ang = np.deg2rad(end_yaw)
    if abs(sv) < 0.01:
        sv = 0.01
    time, x, y, yaw, v, a, j = quintic_polynomials_planner(\
        start_point.x, start_point.y, start_ang, 
        end_point.x, end_point.y, end_ang, dt, 
        sv=sv, max_accel=0.2)
    path = Path()
    if len(time) == 0:
        return path
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "odom"
    for i, t in enumerate(time):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = path.header.stamp + rospy.Duration.from_sec(t)
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = x[i]
        pose_stamped.pose.position.y = y[i]
        pose_stamped.pose.position.z = 0
        rotation_matrix = tf.transformations.rotation_matrix((yaw[i]), (0, 0, 1))
        result_q = tf.transformations.quaternion_from_matrix(rotation_matrix)
        quaternion = Quaternion()
        quaternion.x = result_q[0]
        quaternion.y = result_q[1]
        quaternion.z = result_q[2]
        quaternion.w = result_q[3]
        pose_stamped.pose.orientation = quaternion
        path.poses.append(pose_stamped)   
    line_start = np.array([end_point.x, end_point.y, 0])
    line_end = np.array([straight_end.x, straight_end.y, 0])
    dist = int(np.linalg.norm(line_end - line_start)*20 + 1)
    x_points = np.linspace(line_start[0], line_end[0], dist)
    y_points = np.linspace(line_start[1], line_end[1], dist)
    z_points = np.linspace(line_start[2], line_end[2], dist)
    # points = np.linspace(line_start, line_end, dist)
    points = np.vstack((x_points, y_points, z_points)).T

    for point in points:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = 0
        quaternion = Quaternion()
        quaternion.x = result_q[0]
        quaternion.y = result_q[1]
        quaternion.z = result_q[2]
        quaternion.w = result_q[3]
        pose_stamped.pose.orientation = quaternion
        path.poses.append(pose_stamped)
    return path

def callback_arucoTF(tf_data):
    global Aurco_tf, callback_trigger
    callback_trigger = True
    # Aurco_tf[0] = tf_data
    Aurco_tf = tf_data



def callback_odomTF(sub_odom):
    global robot_odom
    robot_odom = sub_odom
    
def callback_tracking_state(msg):
    global backup_pose, tracking_state, last_plan_info
    if not msg.data == tracking_state:
        tracking_state = msg.data
        if tracking_state == "NOT_WORKING":
            backup_pose = Point()
        elif tracking_state == "ARRIVED":
            del last_plan_info["last_plan"].poses[:]
            last_plan_info['across_displacement'] = False
            last_plan_info['displacement_index'] = 0
            rospy.logwarn("reset plan info")
            
def callback_localplan(msg):
    global last_plan_info
    last_plan_info['last_plan'] = msg
    if last_plan_info['displacement_index'] >= len(msg.poses):
        last_plan_info['across_displacement'] = True
        rospy.logwarn("cross the line")

def get_rotation_angle(p1, p2):
    v = (p2[0]-p1[0], p2[1]-p1[1])
    # 벡터의 노름 계산
    norm_v = math.sqrt(v[0]**2 + v[1]**2)
    # 단위 벡터로 변환
    unit_v = (v[0]/norm_v, v[1]/norm_v)

    # x축 단위 벡터
    x_unit = (1, 0)

    # 두 벡터의 내적 계산
    dot_product = x_unit[0]*unit_v[0] + x_unit[1]*unit_v[1]

    # 각도 계산
    if dot_product == 0:
        angle = 90
    else:
        angle = math.degrees(math.acos(dot_product))

    return angle

def cal_plan(displacement, marker_displacement):
    def getEuclidianDistance(new_point, backup_point):
        dx = new_point.x - backup_point.x
        dy = new_point.y - backup_point.y
        dist = math.hypot(dy, dx)
        return dist
    
    global Aurco_tf, robot_odom, backup_pose
    global debug_pub
    isChange = True
    debug_point = PointStamped()
    debug_point.header.stamp = robot_odom.header.stamp
    debug_point.header.frame_id = robot_odom.header.frame_id
    # if len(Aurco_tf.transforms) != 0 and robot_odom.header.frame_id != "":
    if (Aurco_tf) != None and robot_odom.header.frame_id != "":
        tf_trans = Aurco_tf.transform.translation
        point_origin = Point(x=-marker_displacement, y=0., z=0.)
        point = Point(x=-(displacement+marker_displacement), y=0., z=0)

        origin_quaternion = Aurco_tf.transform.rotation
        origin_translation = (tf_trans.x, tf_trans.y, tf_trans.z)
        origin_matrix = np.dot(tf.transformations.translation_matrix(origin_translation), tf.transformations.quaternion_matrix\
                        ([origin_quaternion.x, origin_quaternion.y, origin_quaternion.z, origin_quaternion.w]))
        
        origin_point = np.array([point_origin.x, point_origin.y, point_origin.z, 1]) 
        origin_new = np.dot(origin_matrix, origin_point) # marker origin point(base odom)
        origin_pos = Point(x=origin_new[0], y=origin_new[1], z=origin_new[2]) # marker origin point(base odom)

        v = np.array([point.x, point.y, point.z, 1]) # offset point
        v_new = np.dot(origin_matrix,v) # offset point
        point_new = Point(x=v_new[0], y=v_new[1], z=v_new[2]) # offset point
        
        # if (getEuclidianDistance(point_new, backup_pose) < 0.02):
        #     isChange = False
        #     return Path(), isChange
        backup_pose = point_new

        start_point = Point(x=robot_odom.pose.pose.position.x, y=robot_odom.pose.pose.position.y, z=0.) # now robot odom position
        q = (
            robot_odom.pose.pose.orientation.x,
            robot_odom.pose.pose.orientation.y,
            robot_odom.pose.pose.orientation.z,
            robot_odom.pose.pose.orientation.w
        )
        q_tf = (origin_quaternion.x, origin_quaternion.y, origin_quaternion.z, origin_quaternion.w)
        robot_angle = tf.transformations.euler_from_quaternion(q)
        tf_angle = tf.transformations.euler_from_quaternion(q_tf)
        debug_point.point = point_new
        debug_pub.publish(debug_point)
        sv = math.hypot(robot_odom.twist.twist.linear.x, robot_odom.twist.twist.linear.y)
        return compute_plan(start_point, np.rad2deg(robot_angle[2]), 
                            point_new, np.rad2deg(tf_angle[2]), 
                            origin_pos, q,
                            sv, 0.2), isChange # get_rotation_angle(v_new,origin_new)
    else: return Path(), isChange

def sub_TF():
    global callback_trigger, backup_pose
    global last_plan_info, tracking_state
    global debug_pub
    callback_trigger = False
    backup_pose = Point()
    last_plan_info = {}
    last_plan_info['across_displacement'] = False
    last_plan_info['last_plan'] = Path()
    last_plan_info['displacement_index'] = 0
    tracking_state = "NOT_WORKING"
    rospy.init_node('subscribeTF', anonymous=True)
    path_displacement = float(rospy.get_param("~path_displacement"))
    robot_size = float(rospy.get_param("~robot_size"))
    marker_displacement = float(rospy.get_param("~marker_displacement"))
    print("path_displacement : ", path_displacement)
    print("robot_size : ", robot_size)
    print("marker_displacement : ", marker_displacement)
    # rospy.Subscriber("tf_list", TFMessage, callback_arucoTF)
    rospy.Subscriber("filtered_tf", TransformStamped, callback_arucoTF)
    rospy.Subscriber("odom", Odometry, callback_odomTF)
    rospy.Subscriber("mpc_state", String, callback_tracking_state)
    rospy.Subscriber("nmpc_ros/local_plan", Path, callback_localplan)
    pub = rospy.Publisher('compute_Path', Path, queue_size=1)
    debug_pub = rospy.Publisher("debug_point", PointStamped, queue_size=1)
    while not rospy.is_shutdown():
        if not callback_trigger:
            rospy.sleep(0.1)
            continue
        plan,isChange = cal_plan(path_displacement, robot_size+marker_displacement)
        if len(plan.poses) != 0:
            if isChange:
                pub.publish(plan)
        callback_trigger = False
        rospy.sleep(0.1)
    

if __name__=='__main__':
    sub_TF()
