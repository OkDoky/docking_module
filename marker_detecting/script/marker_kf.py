#! /usr/bin/python
from pykalman import KalmanFilter
import numpy as np
import time
import math
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def normalizeAngle(val, min_val, max_val):
	norm = 0.
	if val>=min_val:
		norm = min_val + math.fmod((val-min_val),(max_val-min_val))
	else:
		norm = max_val - math.fmod((min_val-val),(max_val-min_val))
	return norm


class KalmanTrack():
    def __init__(self, num_train_sample):
        # rospy.init_node('detecting_marker', anonymous=True)
    
        # self.filtered_tf_pub = rospy.Publisher("filtered_tf", TransformStamped, queue_size=1)
        # self.filtered_pose_pub = rospy.Publisher("filtered_pose", PoseStamped, queue_size=1)
        # self.tf_list_callback = rospy.Subscriber("tf_list", TFMessage, self.tf_cb)
    
        # self.marker_tf = TransformStamped()
        # self.br = TransformBroadcaster()
        
        # self.marker_id = int(rospy.get_param("/marker_id", 7))
        # rospy.logwarn("[KFmarker] marker id is : %s"%self.marker_id)
        
        self.measurements = np.empty((0,3))
        
        self.time_before = time.time()
        self.delta_t = 0.05
        self.default_delta_t = 0.04

        # self.num_train_sample = rospy.get_param('num_train_sample',10)
        self.num_train_sample = num_train_sample

        self.init_trained = False
        self.x_now = None
        self.P_now = None

    def __del__(self):
        pass

    def reset(self):
        self.measurements = np.empty((0,3))
        self.time_before = time.time()

        self.init_trained = False
        self.x_now = None
        self.P_now = None
        
    def tf_cb(self, msg):
        if bool(rospy.get_param("reset_marker", False)):
            rospy.set_param("reset_marker", False)
            self.reset()
        if not len(msg.transforms) == 0:
            for _tf in msg.transforms:
                if int(_tf.child_frame_id) == self.marker_id:
                    td = time.time() - self.time_before
                    self.delta_t = min(self.default_delta_t, td)
                    pred = self.prediction(_tf.transform.translation.x,
                                            _tf.transform.translation.y,
                                            euler_from_quaternion([_tf.transform.rotation.x,
                                                                _tf.transform.rotation.y,
                                                                _tf.transform.rotation.z,
                                                                _tf.transform.rotation.w])[2])
                    if pred is not None:
                        self.publish_pose(pred, _tf.header)
    
    def publish_pose(self, pred, header):
        # publish Pose Stamped
        filtered_pose = PoseStamped()
        filtered_pose.header = header
        filtered_pose.pose.position.x = pred[0]
        filtered_pose.pose.position.y = pred[2]
        angle = normalizeAngle(pred[4], -math.pi, math.pi)
        q = quaternion_from_euler(0, 0, angle)
        filtered_pose.pose.orientation.x = q[0]
        filtered_pose.pose.orientation.y = q[1]
        filtered_pose.pose.orientation.z = q[2]
        filtered_pose.pose.orientation.w = q[3]
        self.filtered_pose_pub.publish(filtered_pose)
        
        # publish transformStamped
        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = str(self.marker_id)
        transform.transform.translation.x = pred[0]
        transform.transform.translation.y = pred[2]
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.filtered_tf_pub.publish(transform)
        
        
    def init_train(self):
        try:
            initial_state_mean = [self.measurements[0, 0], 0,
                                  self.measurements[0, 1], 0,
                                  self.measurements[0, 2], 0]
            delta_t = self.delta_t
            rospy.logwarn("[KFmarker] dt : %s"%delta_t)
            # transition_matrix = [[1, delta_t,   0,       0,     0,      0],
            #                      [0,       1,   0,       0,     0,      0],
            #                      [0,       0,   1, delta_t,     0,      0],
            #                      [0,       0,   0,       1,     0,      0],
            #                      [0,       0,   0,       0,     1,delta_t],
            #                      [0,       0,   0,       0,     0,      1]]
            transition_matrix = [[1,       0,   0,       0,     0,      0],
                                 [0,       1,   0,       0,     0,      0],
                                 [0,       0,   1,       0,     0,      0],
                                 [0,       0,   0,       1,     0,      0],
                                 [0,       0,   0,       0,     1,      0],
                                 [0,       0,   0,       0,     0,      1]]

            observation_matrix = [[1, 0, 0, 0, 0, 0],
                                  [0, 0, 1, 0, 0, 0],
                                  [0, 0, 0, 0, 1, 0]]


            self.kf1 = KalmanFilter(transition_matrices = transition_matrix,
                                    observation_matrices = observation_matrix,
                                    initial_state_mean = initial_state_mean,
                                    em_vars=['transition_covariance', 'initial_state_covariance'])

            self.kf1 = self.kf1.em(self.measurements, n_iter=5)
            (filtered_state_means, filtered_state_covariances) = self.kf1.filter(self.measurements)

            self.x_now = filtered_state_means[-1, :]
            self.P_now = filtered_state_covariances[-1, :]

            self.kf3 = KalmanFilter(transition_matrices = transition_matrix,
                                    observation_matrices = observation_matrix,
                                    initial_state_mean = initial_state_mean,
                                    observation_covariance = 10*self.kf1.observation_covariance,
                                    em_vars=['transition_covariance', 'initial_state_covariance'])

            self.kf3 = self.kf3.em(self.measurements, n_iter=5)
            (filtered_state_means, filtered_state_covariances) = self.kf3.filter(self.measurements)

            self.init_trained = True
        except Exception as e:
            rospy.logerr("[KalmanTrack] init_training is failed...(%s)"%e)
            


    def update_filter(self, new_x, new_y, new_th):
        self.time_before = time.time()
        (self.x_now, self.P_now) = self.kf3.filter_update(filtered_state_mean = self.x_now,
                                                        filtered_state_covariance = self.P_now,
                                                        observation = [new_x,new_y,new_th])

    def prediction(self, z_means):
        pos_x = z_means[0]
        pos_y = z_means[1]
        pos_th = z_means[-1]
        if self.measurements.shape[0] <= self.num_train_sample:
            self.measurements = np.vstack((self.measurements,[pos_x, pos_y, pos_th]))
            if self.measurements.shape[0] == self.num_train_sample:
                self.init_train()
        elif self.init_trained:
            self.update_filter(pos_x,pos_y,pos_th)
        else:
            self.init_train()
            rospy.logerr("[KalmanTrack] unknown errs, init_trained: %d/train_sample: %d"%(self.init_trained,self.measurements.shape[0]))
        if self.x_now is None:
            return None
        else:
            pred = np.array([self.x_now[0], self.x_now[2], 0, 0, 0, self.x_now[4]])
            return pred

if __name__ == '__main__':
    filter = KalmanTrack()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

