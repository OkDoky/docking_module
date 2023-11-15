#! /usr/bin/python
import numpy as np
from numpy.linalg import inv

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf.transformations import *

class Filter():
    def __init__(self):
        rospy.init_node('detecting_marker', anonymous=True)
    
        self.pub = rospy.Publisher("filtered_tf", TransformStamped, queue_size=1)
        self.filtered_pose_pub = rospy.Publisher("filtered_pose", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("tf_list", TFMessage, self.cb)
    
        self.marker_tf = TransformStamped()
        self.br = TransformBroadcaster()
        self.swich = False
        self.marker_id = rospy.get_param("/marker_id", 7)
        
        self.A = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        self.Q = 0.01 * np.eye(6)
        self.R = 1000.0 * np.eye(6)
        
        self.x = None
        self.P = 1.0 * np.eye(6)
        
    def cb(self, msg):
        if self.swich:
            if bool(rospy.get_param("reset_marker", False)):
                rospy.set_param("reset_marker", False)
                self.reset_kf()
            if len(msg.transforms):
                for tf in msg.transforms:
                    if int(tf.child_frame_id[-1]) == self.marker_id:
                        self.marker_tf = msg.transforms[0]
                        euler = euler_from_quaternion([self.marker_tf.transform.rotation.x,
                                                        self.marker_tf.transform.rotation.y,
                                                        self.marker_tf.transform.rotation.z,
                                                        self.marker_tf.transform.rotation.w])
                        z_meas = np.array([self.marker_tf.transform.translation.x,
                                        self.marker_tf.transform.translation.y,
                                        self.marker_tf.transform.translation.z,
                                        euler[0],
                                        euler[1],
                                        euler[2]])
                        self.kalman_filter(z_meas)
                        filtered_pose = PoseStamped()
                        filtered_pose.header.stamp = tf.header.stamp
                        filtered_pose.header.frame_id = tf.header.frame_id
                        quaternion = quaternion_from_euler(self.x[3], self.x[4], self.x[5])
                        self.marker_tf.transform.translation.x = self.x[0]
                        self.marker_tf.transform.translation.y = self.x[1]
                        self.marker_tf.transform.translation.z = self.x[2]
                        self.marker_tf.transform.rotation.x = quaternion[0]
                        self.marker_tf.transform.rotation.y = quaternion[1]
                        self.marker_tf.transform.rotation.z = quaternion[2]
                        self.marker_tf.transform.rotation.w = quaternion[3]
                        filtered_pose.pose.position.x = self.x[0]
                        filtered_pose.pose.position.y = self.x[1]
                        filtered_pose.pose.orientation.x = quaternion[0]
                        filtered_pose.pose.orientation.y = quaternion[1]
                        filtered_pose.pose.orientation.z = quaternion[2]
                        filtered_pose.pose.orientation.w = quaternion[3]
                        self.pub.publish(self.marker_tf)
                        self.filtered_pose_pub.publish(filtered_pose)
                        self.br.sendTransform(self.marker_tf)
        else:
            if len(msg.transforms):
                for tf in msg.transforms:
                    if int(tf.child_frame_id[-1]) == self.marker_id:
                        self.marker_tf = msg.transforms[0]
                        euler = euler_from_quaternion([self.marker_tf.transform.rotation.x,
                                                        self.marker_tf.transform.rotation.y,
                                                        self.marker_tf.transform.rotation.z,
                                                        self.marker_tf.transform.rotation.w])
                        self.x = np.array([self.marker_tf.transform.translation.x,
                                        self.marker_tf.transform.translation.y,
                                        self.marker_tf.transform.translation.z,
                                        euler[0],
                                        euler[1],
                                        euler[2]])
                        self.swich = True
                        
    def kalman_filter(self, z_meas):  
        if self.x is not None:
            x_pred = np.dot(self.A, self.x)
            P_pred = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

            K = np.dot(np.dot(P_pred, self.H.T), inv(np.dot(np.dot(self.H, P_pred), self.H.T) + self.R))

            self.x = x_pred + np.dot(K, (z_meas - np.dot(self.H, x_pred)))

            self.P = P_pred - np.dot(np.dot(K, self.H), P_pred)
        else:
            self.x = z_meas
    
    def reset_kf(self):
        self.x = None
        self.P = 1.0 * np.eye(6)

if __name__ == '__main__':
    filter = Filter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
