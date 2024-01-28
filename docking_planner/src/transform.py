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
from geometry_msgs.msg import PolygonStamped, Point32, Point, Quaternion, PoseStamped, TransformStamped, Pose2D
from tf.transformations import quaternion_matrix, quaternion_from_matrix, rotation_matrix
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

def normalizeAngle(angle, min, max):
    if angle > max:
        angle -= 2 * math.pi
    elif angle < min:
        angle += 2 * math.pi
    return angle

def normalizeAngles(angles, min_angle, max_angle):
    angles = np.where(angles > max_angle, angles - 2 * math.pi, angles)
    angles = np.where(angles < min_angle, angles + 2 * math.pi, angles)
    return angles

def is_near_target_value(target, value, epsilon=1e-9):
    return abs(target-value) < epsilon

def dot_product(v,w):
    return v.x * w.x + v.y * w.y

def magnitude(v):
    return math.sqrt(v.x ** 2 + v.y ** 2)

def has_cross_line(a, b, c):
    """check angle between a,b,c point

    Args:
        a (Pose2D): current robot pose
        b (Pose2D): target straight line start pose
        c (Pose2D): target pose

    Returns:
        Bool : return True if robot cross the line start with straight
    """
    ba = Pose2D(x=a.x - b.x, y=a.y - b.y, theta=0.0)
    bc = Pose2D(x=c.x - b.x, y=c.y - b.y, theta=0.0)

    dot_prod = dot_product(ba, bc)
    angle = math.acos(dot_prod / (magnitude(ba) * magnitude(bc)))

    angle_deg = math.degrees(angle)
    # print("[Transform] angle deg : %s"%abs(angle_deg))
    return abs(angle_deg) < 90.

def move_pose(target_pose, dist=0.1):
    dx = math.cos(target_pose.theta) * dist
    dy = math.sin(target_pose.theta) * dist
    
    result_pose = Pose2D()
    result_pose.x = target_pose.x + dx
    result_pose.y = target_pose.y + dy
    result_pose.theta = target_pose.theta
    # print("[MOvePOse] target pose : (%s, %s), result : (%s, %s)"%(target_pose.x, target_pose.y,
    #                                              result_pose.x, result_pose.y))
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

def debug_has_cross_line(a,b,c):
    global polygon_pub
    poly = PolygonStamped()
    poly.header.stamp = rospy.Time.now()
    poly.header.frame_id = "odom"
    pa = Point32(x=a.x, y=a.y, z=0.0)
    pb = Point32(x=b.x, y=b.y, z=0.0)
    pc = Point32(x=c.x, y=c.y, z=0.0)
    poly.polygon.points.append(pa)
    poly.polygon.points.append(pb)
    poly.polygon.points.append(pc)
    polygon_pub.publish(poly)
    
def compute_plan(start_pose,  end_pose, 
                 straight_end, frame, sv, ev, dt=0.1):
    """_summary_

    Args:
        start_pose (Pose2D): current pose of robot - odom frame
        end_pose (Pose2D): pose of end of polynomial plan - odom frame
        straight_end (Pose2D): target pose(end of last path) - odom frame
        frame (string): 
        sv (float): starting velocity(current velocity)
        ev (float): velocity with tracking linear path
        dt (float, optional): time difference. Defaults to 0.1.

    Returns:
        Path: calculated path
    """
    global heading_front, debug_mode
    
    if abs(sv) < 0.01:
        sv = 0.01
    if debug_mode:
        debug_has_cross_line(start_pose, end_pose, straight_end)
    end_pose = move_pose(end_pose, dist=0.05)
    _has_cross_line = has_cross_line(start_pose, end_pose, straight_end)
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = frame
    result_q = np.zeros(4)
    if not _has_cross_line:
        rospy.logdebug("[Transform] Didn't Cross the Line")
        time, x, y, yaw, v, a, j = quintic_polynomials_planner(\
            start_pose.x, start_pose.y, 
            start_pose.theta if heading_front else normalizeAngle(start_pose.theta+math.pi, -math.pi, math.pi),
            end_pose.x, end_pose.y, 
            end_pose.theta, 
            dt, sv=sv, gv=ev, max_accel=0.2)
        # if not heading_front:
        #     yaw = np.array(yaw)
        #     yaw = normalizeAngles(yaw+math.pi, -math.pi, math.pi).tolist()
            
        for i, t in enumerate(time):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path.header.stamp + rospy.Duration.from_sec(t)
            pose_stamped.header.frame_id = frame
            pose_stamped.pose.position.x = x[i]
            pose_stamped.pose.position.y = y[i]
            pose_stamped.pose.position.z = 0
            _rotation_matrix = rotation_matrix((yaw[i]), (0, 0, 1))
            result_q = quaternion_from_matrix(_rotation_matrix)
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
    rot = rotation_matrix((end_pose.theta), (0, 0, 1))
    linear_q = quaternion_from_matrix(rot)
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
    global path_displacement, robot_size, marker_displacement, heading_front
    rospy.logwarn("[TrackingStateCallback] %s"%msg.data)
    if not msg.data == tracking_state:
        tracking_state = msg.data
        if tracking_state in ["NOT_WORKING", "ARRIVED"]:
            backup_pose = Point()
            path_displacement = float(rospy.get_param("~path_displacement"))
            robot_size = float(rospy.get_param("~robot_size"))
            marker_displacement = float(rospy.get_param("~marker_displacement"))
            heading_front = bool(rospy.get_param("~heading_front", True))

def cal_plan(displacement, marker_displacement, aruco_tf):
    global robot_odom, backup_pose
    global pub
    if (aruco_tf) != None and robot_odom.header.frame_id != "":
        ## for calculate offset end point from marker pose
        tf_trans = aruco_tf.transform.translation
        point_origin = Point(x=-marker_displacement, y=0., z=0.)
        point = Point(x=-(displacement+marker_displacement), y=0., z=0)

        origin_quaternion = aruco_tf.transform.rotation
        origin_translation = (tf_trans.x, tf_trans.y, tf_trans.z)
        origin_matrix = np.dot(tf.transformations.translation_matrix(origin_translation), quaternion_matrix\
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
        q_tf = (origin_quaternion.x, origin_quaternion.y, origin_quaternion.z, origin_quaternion.w)
        
        tf_angle = tf.transformations.euler_from_quaternion(q_tf)
        target_angle = tf_angle[2]
        # if not heading_front:
        #     target_angle = normalizeAngle(target_angle + math.pi, -math.pi, math.pi)
        #     robot_angle = normalizeAngle(robot_angle + math.pi, -math.pi, math.pi)
        current_robot_pose = Pose2D(x=robot_odom.pose.pose.position.x, y=robot_odom.pose.pose.position.y, theta=robot_angle) # now robot odom position
        pose_new = Pose2D(x=point_new.x, y=point_new.y, theta=target_angle)
        origin_pos = Pose2D(x=origin_new[0], y=origin_new[1], theta=target_angle) # marker origin point(base odom)
        sv = math.hypot(robot_odom.twist.twist.linear.x, robot_odom.twist.twist.linear.y)
        result_path = compute_plan(current_robot_pose,
                            pose_new, 
                            origin_pos, robot_odom.header.frame_id,
                            sv, 0.2, dt=0.1)
        pub.publish(result_path)
        

def sub_TF():
    global callback_trigger, backup_pose
    global tracking_state
    global pub, polygon_pub, debug_mode
    global cm
    callback_trigger = False
    backup_pose = Point()
    tracking_state = "NOT_WORKING"
    rospy.init_node('subscribeTF')
    cm = CallbackManager()
    cm.register("planning", cal_plan)
    global path_displacement, robot_size, marker_displacement, heading_front
    path_displacement = float(rospy.get_param("~path_displacement"))
    robot_size = float(rospy.get_param("~robot_size"))
    marker_displacement = float(rospy.get_param("~marker_displacement"))
    heading_front = bool(rospy.get_param("~heading_front", True))
    debug_mode = bool(rospy.get_param("~debug_mode", False))
    print("path_displacement : ", path_displacement)
    print("robot_size : ", robot_size)
    print("marker_displacement : ", marker_displacement)
    print("heading_front : ", heading_front)
    rospy.Subscriber("filtered_tf", TransformStamped, callback_arucoTF)
    rospy.Subscriber("odom", Odometry, callback_odomTF)
    rospy.Subscriber("docking/status", String, callback_tracking_state)
    pub = rospy.Publisher('compute_Path', Path, queue_size=1)
    polygon_pub = rospy.Publisher("angle_check", PolygonStamped, queue_size=1)
    rospy.spin()
    

if __name__=='__main__':
    sub_TF()
