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
from std_msgs.msg import String
from std_srvs.srv import SetBoolResponse, SetBool
from shapely.geometry import Polygon, Point

D_WATTING = 0
D_MOVING = 1
D_STOPPED = 2
D_DONE = 3
D_ERROR = 4

status_dict = {D_WATTING: "WAITTING", D_MOVING: "MOVING", D_STOPPED: "STOPPED", D_DONE: "ARRIVED", D_ERROR: "ERROR"}

class DockingOUT:
    def __init__(self):
        rospy.init_node("DockingOut")
        self.footprint = None
        self.rpose = Pose2D()
        self.scan = LaserScan()
        self.default_speed = 0.2
        self.subs = []
        self.srvs = []
        self.pubs = {}
        self.subs.append(rospy.Subscriber("scan", LaserScan, self._scan_callback))
        self.subs.append(rospy.Subscriber("odom", Odometry, self._odom_callback))
        self.srvs.append(rospy.Service("set_docking_out", SetBool, self.docking_trigger))
        self.pubs['cmd'] = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pubs['state'] = rospy.Publisher("docking_out/status", String, queue_size=1)
        self.direction = -1 if bool(rospy.get_param("~go_back", True)) else 1
        self.is_running = False
        self.is_paused = False
        self.is_detected = False
        self.is_canceled = False
        self.status = D_WATTING
        rospy.spin()
    
    def docking_trigger(self, req):
        if req.data:
            dist = float(rospy.get_param("~dist", 0.5))
            self.direction = -1 if bool(rospy.get_param("~go_back", True)) else 1
            if self.footprint == None:
                polygon = np.array(eval(rospy.get_param("move_base/local_costmap/footprint", [[-0.105,-0.105],[-0.105,0.105],[0.105,0.105],[0.105,-0.105]])))
                _sfp = [(point[0], point[1]) for point in polygon]
                self.footprint = Polygon(_sfp)
            self.run(dist)
        else:
            self.is_canceled = True

        res = SetBoolResponse()
        res.success = True
        return res
    
    def run(self, dist):
        self.is_running = True
        self.is_paused = False
        self.is_detected = False
        self.is_canceled = False
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
            self.is_canceled = False
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
        
        cmd = Twist()
        while not rospy.is_shutdown():
            try:
                if self.is_paused:
                    continue
                current_p = deepcopy(self.rpose)
                if self._dist_btw_2_point(start_p, current_p) > dist or self.is_canceled:
                    self.status = D_DONE
                    rospy.logwarn("[Done] moved %f m"%(self._dist_btw_2_point(start_p, current_p)))
                    cmd.linear.x = 0.0
                    self.pubs['cmd'].publish(cmd)
                    break
                scan = self.scan
                # self.is_detected = self._scan_in_polygon(scan)
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
                self.pubs['state'].publish(status_dict[self.status])

        
    def _scan_in_polygon(self, scan):
        distances = np.array(scan.ranges)
        points = np.column_stack(([r * math.cos(scan.angle_min + i * scan.angle_increment) for i, r in enumerate(scan.ranges)],
                                [r * math.sin(scan.angle_min + i * scan.angle_increment) for i, r in enumerate(scan.ranges)]))

        inside_mask = np.array([self.footprint.contains(Point(pt)) for pt in points])
        try:
            distances_inside = distances[inside_mask]
        except IndexError:
            return False
        if len(distances_inside)>0:
            rospy.logwarn("[DockingOut] %d point is inside footprint"%len(distances_inside))
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
