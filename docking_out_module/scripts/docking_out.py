#!/usr/bin/python
import rospy
import math
import time
import numpy as np
import traceback
from threading import Thread, Event
from copy import deepcopy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBoolResponse, SetBool
from shapely.geometry import Polygon, Point

D_WATTING = 0
D_MOVING = 1
D_STOPPED = 2
D_DONE = 3
D_ERROR = 4

class DockingOUT:
    def __init__(self):
        rospy.init_node("DockingOut")
        polygon = np.array(eval(rospy.get_param("move_base/local_costmap/footprint", [[-0.105,-0.105],[-0.105,0.105],[0.105,0.105],[0.105,-0.105]])))
        rospy.logwarn(polygon)
        _sfp = [(point[0], point[1]) for point in polygon]
        self.footprint = Polygon(_sfp)
        self.rpose = Pose2D()
        self.scan = LaserScan()
        self.direction = -1
        self.default_speed = 0.2
        self.subs = []
        self.srvs = []
        self.pubs = {}
        self.subs.append(rospy.Subscriber("scan", LaserScan, self._scan_callback))
        self.subs.append(rospy.Subscriber("odom", Odometry, self._odom_callback))
        self.srvs.append(rospy.Service("set_docking_out", SetBool, self.docking_trigger))
        self.pubs['cmd'] = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.is_running = False
        self.is_paused = False
        self.is_detected = False
        self.status = D_WATTING
        rospy.spin()
    
    def docking_trigger(self, req):
        dist = float(rospy.get_param("docking_out/dist", 0.5))
        self.run(dist)
        res = SetBoolResponse()
        res.success = True
        return res
    
    def run(self, dist):
        self.is_running = True
        rospy.logwarn("start to make thread")
        _docking_out = Thread(target=self.execute_callback, args=(dist,))
        _docking_out.daemon = True
        _docking_out.start()
    
    # @staticmethod    
    def is_done(self):
        if self.status == D_DONE:
            self.status = D_WATTING
            self.is_running = False
            self.is_paused = False
            self.is_detected = False
            return True
        else:
            return False
        
    # @staticmethod
    def set_pause(self):
        self.is_paused = True
        
    # @staticmethod
    def set_resume(self):
        self.is_paused = False
                
    def execute_callback(self, dist):
        start_p = deepcopy(self.rpose)
        current_p = deepcopy(self.rpose)
        self.status = D_MOVING
        
        rospy.logwarn("init thread, start point : %s, end point : %s"%(start_p, current_p))
        cmd = Twist()
        while not rospy.is_shutdown():
            try:
                if self.is_paused:
                    continue
                current_p = deepcopy(self.rpose)
                if self._dist_btw_2_point(start_p, current_p) > dist:
                    self.status = D_DONE
                    rospy.logwarn("[Done] start p : %s, end p : %s"%(start_p, current_p))
                    cmd.linear.x = 0.0
                    self.pubs['cmd'].publish(cmd)
                    break
                scan = self.scan
                self.is_detected = self._scan_in_polygon(scan)
                if self.is_detected:
                    self.status = D_STOPPED
                    cmd.linear.x = 0.0
                    self.pubs['cmd'].publish(cmd)
                else:
                    self.status = D_MOVING
                    cmd.linear.x = self.default_speed * self.direction
                    self.pubs['cmd'].publish(cmd)
                
            except Exception:
                rospy.logwarn("except %s"%traceback.format_exc())
            finally:
                rospy.sleep(0.1)
                rospy.logwarn("current state : %d"%self.status)
        
    def _scan_in_polygon(self, scan):
        distances = np.array(scan.ranges)
        points = np.column_stack(([r * math.cos(scan.angle_min + i * scan.angle_increment) for i, r in enumerate(scan.ranges)],
                                [r * math.sin(scan.angle_min + i * scan.angle_increment) for i, r in enumerate(scan.ranges)]))

        inside_mask = np.array([self.footprint.contains(Point(pt)) for pt in points])
        distances_inside = distances[inside_mask]
        if len(distances_inside)>0:
            return True
        else:
            return False
        
    def _dist_btw_2_point(self, start, end):
        return math.hypot(start.x - end.x, start.y - end.y)
        
    def _odom_callback(self, msg):
        self.rpose.x = msg.pose.pose.position.x
        self.rpose.y = msg.pose.pose.position.y  
     
    def _scan_callback(self, msg):
        if not self.is_running:
            return
        if self.is_paused:
            return
        self.scan = msg
 
if __name__ == "__main__":
    _cls = DockingOUT()
    print("done dockingout")