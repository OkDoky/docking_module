#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import tf
import math
import numpy as np
from copy import copy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, QuaternionStamped, Point, Quaternion, PoseStamped, TransformStamped, Pose2D
from tf.transformations import quaternion_matrix, quaternion_from_matrix
sys.path.append(os.path.dirname(__file__))
print(os.path.dirname(__file__))

from planner import quintic_polynomials_planner

class CallbackManager:
    def __init__(self):
        self.callbacks = {}
    
    def register(self, key, func):
        if key not in self.callbacks:
            self.callbacks[key] = []
        self.callbacks[key].append(func)
        rospy.loginfo("[CallbackManager] register { key : %s, func : %s}"%(key, str(func)))
    
    def trigger(self, key, *args, **kwargs):
        if key in self.callbacks:
            for callback in self.callbacks[key]:
                callback(*args, **kwargs)

robot_odom = Odometry()

def is_near_target_value(target, value, epsilon=1e-9):
    return abs(target-value) < epsilon

def has_cross_line(a, b, c):
    """_summary_

    Args:
        a (Pose2D): current robot pose
        b (Pose2D): target straight line start pose
        c (Pose2D): target pose

    Returns:
        Bool : return True if robot cross the line start with straight
    """
    if is_near_target_value(math.pi/2.0, c.theta) or is_near_target_value(-math.pi/2.0, c.theta):
        # 이 경우, 선은 수평선이므로 A의 x 좌표만 비교하면 됨
        return a.x > b.x
    # Calculate the slope of the line perpendicular to the direction at C
    perpendicular_slope = -1 / math.tan(c.theta)

    # The equation of the line: y = mx + c
    # We need to find 'c' (y-intercept) for the line
    line_y_intercept = b.y - perpendicular_slope * b.x

    # Substitute point A into the line equation to determine its position relative to the line
    side = a.y - (perpendicular_slope * a.x + line_y_intercept)

    return side < 0  # Returns True if A is on one side, False if on the other

def move_pose(target_pose, dist=0.1):
    dx = math.cos(target_pose.theta) * dist
    dy = math.sin(target_pose.theta) * dist
    
    result_pose = Pose2D()
    result_pose.x = target_pose.x - dx
    result_pose.y = target_pose.y - dy
    result_pose.theta = target_pose.theta
    return result_pose

def find_nearest_point_on_line(line_start, line_end, pos):
    ls = np.array([line_start.x, line_start.y, line_start.theta])
    le = np.array([line_end.x, line_end.y, line_end.theta])
    p = np.array([pos.x, pos.y, pos.theta])
    line_vec = le - ls
    point_vec = p - ls
    line_len = np.linalg.norm(line_vec)
    line_unitvec = line_vec / line_len
    point_proj = np.dot(point_vec, line_unitvec)
    point_proj = np.clip(point_proj, 0, line_len)
    nrp = ls + line_unitvec * point_proj
    nearest_point = Pose2D(x=nrp[0], y=nrp[1], theta=nrp[2])
    return nearest_point

def compute_plan(start_pose,  end_pose, 
                 straight_end, frame, sv, ev, dt=0.1):
    """_summary_

    Args:
        start_point (Point): current pose of robot - odom frame
        end_point (Point): pose of end of polynomial plan - odom frame
        straight_end (Point): target pose(end of last path) - odom frame
        frame (string): 
        sv (float): starting velocity(current velocity)
        ev (float): velocity with tracking linear path
        dt (float, optional): time difference. Defaults to 0.1.

    Returns:
        Path: calculated path
    """
    
    if abs(sv) < 0.01:
        sv = 0.01
    moved_end_pose = move_pose(end_pose, dist=0.05)
    _has_cross_line = has_cross_line(start_pose, moved_end_pose, end_pose)
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = frame
    result_q = np.zeros(4)
    if not _has_cross_line:
        rospy.logdebug("[Transform] Didn't Cross the Line")
        time, x, y, yaw, v, a, j = quintic_polynomials_planner(\
            start_pose.x, start_pose.y, start_pose.theta, 
            end_pose.x, end_pose.y, end_pose.theta, dt, 
            sv=sv, gv=ev, max_accel=0.2)
        for i, t in enumerate(time):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path.header.stamp + rospy.Duration.from_sec(t)
            pose_stamped.header.frame_id = frame
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
        new_end_point = copy(end_pose)
    else: # already cross the line..
        rospy.logdebug("[Transform] Cross the Line")
        new_end_point = find_nearest_point_on_line(end_pose, straight_end, start_pose)
    line_start = np.array([new_end_point.x, new_end_point.y, 0])
    line_end = np.array([straight_end.x, straight_end.y, 0])
    dist = int(np.linalg.norm(line_end - line_start)*20 + 1)
    x_points = np.linspace(line_start[0], line_end[0], dist)
    y_points = np.linspace(line_start[1], line_end[1], dist)
    z_points = np.linspace(line_start[2], line_end[2], dist)
    # points = np.linspace(line_start, line_end, dist)
    points = np.vstack((x_points, y_points, z_points)).T
    rot = tf.transformations.rotation_matrix((end_pose.theta), (0, 0, 1))
    linear_q = tf.transformations.quaternion_from_matrix(rot)
    for point in points:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = 0
        quaternion = Quaternion()
        quaternion.x = linear_q[0]
        quaternion.y = linear_q[1]
        quaternion.z = linear_q[2]
        quaternion.w = linear_q[3]
        pose_stamped.pose.orientation = quaternion
        path.poses.append(pose_stamped)
    return path

def callback_arucoTF(tf_data):
    global callback_trigger, cm
    global path_displacement, robot_size, marker_displacement
    callback_trigger = True
    ar_tf = tf_data
    cm.trigger("planning", 
               path_displacement, robot_size + marker_displacement,
               ar_tf)

def callback_odomTF(sub_odom):
    global robot_odom
    robot_odom = sub_odom
    
def callback_tracking_state(msg):
    global backup_pose, tracking_state
    if not msg.data == tracking_state:
        tracking_state = msg.data
        if tracking_state in ["NOT_WORKING", "ARRIVED"]:
            backup_pose = Point()

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

def cal_plan(displacement, marker_displacement, aruco_tf):
    def getEuclidianDistance(new_point, backup_point):
        return math.hypot(new_point.y - backup_point.y, new_point.x - backup_point.x)
    
    global robot_odom, backup_pose
    global pub
    if (aruco_tf) != None and robot_odom.header.frame_id != "":
        ## for calculate offset end point from marker pose
        tf_trans = aruco_tf.transform.translation
        point_origin = Point(x=-marker_displacement, y=0., z=0.)
        point = Point(x=-(displacement+marker_displacement), y=0., z=0)

        origin_quaternion = aruco_tf.transform.rotation
        origin_translation = (tf_trans.x, tf_trans.y, tf_trans.z)
        origin_matrix = np.dot(tf.transformations.translation_matrix(origin_translation), tf.transformations.quaternion_matrix\
                        ([origin_quaternion.x, origin_quaternion.y, origin_quaternion.z, origin_quaternion.w]))
        
        origin_point = np.array([point_origin.x, point_origin.y, point_origin.z, 1]) 
        origin_new = np.dot(origin_matrix, origin_point) # marker origin point(base odom)
        
        v = np.array([point.x, point.y, point.z, 1]) # offset point
        v_new = np.dot(origin_matrix,v) # offset point
        point_new = Point(x=v_new[0], y=v_new[1], z=v_new[2]) # offset point
        backup_pose = point_new

        q = (
            robot_odom.pose.pose.orientation.x,
            robot_odom.pose.pose.orientation.y,
            robot_odom.pose.pose.orientation.z,
            robot_odom.pose.pose.orientation.w
        )
        robot_angle = tf.transformations.euler_from_quaternion(q)[2]
        current_robot_pose = Pose2D(x=robot_odom.pose.pose.position.x, y=robot_odom.pose.pose.position.y, theta=robot_angle) # now robot odom position
        
        q_tf = (origin_quaternion.x, origin_quaternion.y, origin_quaternion.z, origin_quaternion.w)
        
        tf_angle = tf.transformations.euler_from_quaternion(q_tf)
        pose_new = Pose2D(x=point_new.x, y=point_new.y, theta=tf_angle[2])
        origin_pos = Pose2D(x=origin_new[0], y=origin_new[1], theta=tf_angle[2]) # marker origin point(base odom)
        sv = math.hypot(robot_odom.twist.twist.linear.x, robot_odom.twist.twist.linear.y)
        result_path = compute_plan(current_robot_pose,
                            pose_new, 
                            origin_pos, robot_odom.header.frame_id,
                            sv, 0.2, dt=0.1)
        pub.publish(result_path)
        

def sub_TF():
    global callback_trigger, backup_pose
    global tracking_state
    global pub
    global cm
    callback_trigger = False
    backup_pose = Point()
    tracking_state = "NOT_WORKING"
    rospy.init_node('subscribeTF')
    cm = CallbackManager()
    cm.register("planning", cal_plan)
    global path_displacement, robot_size, marker_displacement
    path_displacement = float(rospy.get_param("~path_displacement"))
    robot_size = float(rospy.get_param("~robot_size"))
    marker_displacement = float(rospy.get_param("~marker_displacement"))
    print("path_displacement : ", path_displacement)
    print("robot_size : ", robot_size)
    print("marker_displacement : ", marker_displacement)
    rospy.Subscriber("filtered_tf", TransformStamped, callback_arucoTF)
    rospy.Subscriber("odom", Odometry, callback_odomTF)
    rospy.Subscriber("mpc_state", String, callback_tracking_state)
    pub = rospy.Publisher('compute_Path', Path, queue_size=1)
    rospy.spin()
    

if __name__=='__main__':
    sub_TF()
