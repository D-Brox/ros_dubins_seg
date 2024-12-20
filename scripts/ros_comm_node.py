#!/usr/bin/env python

import sys
import time
from itertools import combinations
import numpy as np
import yaml
import colorsys


import rospy
from geometry_msgs.msg import Twist,Point
from tf.transformations import quaternion_from_euler

from pycrazyswarm import Crazyswarm

from dubins_seg.srv import *

TAKEOFF_DURATION = 5.0
ADJUST_DURATION = 5.0

class CommNode():

    def __init__(self,cf_yaml, groups,n_robots, freq=60):
        self.swarm = Crazyswarm(crazyflies_yaml=cf_yaml)#,args="--sim --vis mpl")
        # rospy.init_node("communication")
        self.timeHelper = self.swarm.timeHelper
        self.cfs = self.swarm.allcfs.crazyfliesById
        with open(cf_yaml, 'r') as file:
            self.__IDs = {k: v['id'] for (k,v) in enumerate(yaml.safe_load(file)["crazyflies"])}

        self.__n_robots = n_robots
        self.__freq = float(freq)
        self.__x = [0]*self.__n_robots
        self.__y = [0]*self.__n_robots
        self.__theta = [0]*self.__n_robots
        self.__memory = [None]*self.__n_robots
        self.__params = self.load_sim_param()
        self.__start = [False for _ in range(self.__n_robots)]
        self.__groups = groups
        self.__group_list = list(set(groups))
        self.__n_groups = len(self.__group_list)

        hsv = [(x*1.0/self.__n_groups, 1, 1) for x in range(self.__n_groups)]
        self.__colors = [colorsys.hsv_to_rgb(*x) for x in hsv]

        # Topics
        self.__pose_publisher = [None]*self.__n_robots
        for i in range(self.__n_robots):
            self.cfs[self.__IDs[i]].setLEDColor(*(self.__colors[groups[i]]))
            self.__pose_publisher[i] = rospy.Publisher(f"robot_{i}/base_pose_ground_truth", Point, queue_size=10)
            rospy.Subscriber(f"/robot_{i}/next_point", Point, self.callback_point,(i))
        rospy.Service("start",start,lambda _:all(self.__start) and all(self.__start))

        self.__send = [rospy.ServiceProxy(f"/robot_{i}/send_mem_{i}",send_memory,persistent=True) for i in range(self.__n_robots)]
        self.__receive = [rospy.ServiceProxy(f"/robot_{i}/receive_mem_{i}", receive_memory,persistent=True) for i in range(self.__n_robots)]
        rospy.Service("update_i",update_i,self.update_i)

        self.__rate = rospy.Rate(self.__freq)
        rospy.on_shutdown(self.land)

    def main_loop(self):
        self.__params = self.load_sim_param()

        # Takeoff
        self.swarm.allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
        print("takeoff")
        self.timeHelper.sleep(TAKEOFF_DURATION)
        for i in range(self.__n_robots):
            [x,y,_] = self.cfs[self.__IDs[i]].position()
            d = self.__params["d"]
            r = np.sqrt(x**2+y**2)
            rh = round(r/d)*d
            if not rh:
                rh = d
            self.cfs[self.__IDs[i]].goTo(goal = [x*rh/r,y*rh/r,0.5], yaw = 0, duration = ADJUST_DURATION)
        print("adjust")
        self.timeHelper.sleep(ADJUST_DURATION)
        while not rospy.is_shutdown():
            for i in range(self.__n_robots):
                position = self.cfs[self.__IDs[i]].position()
                self.callback_pose(position,i)
            self.receive_all()
            self.swarm.timeHelper.sleep(1/self.__freq)
            self.__rate.sleep()
            self.metric()

    def receive_all(self):
        for i in range(self.__n_robots):
            self.__memory[i] = self.__send[i](i,i)

    def update_i(self,req):
        i = req.i
        j_list = []
        for j in range(self.__n_robots):
            if (self.__x[i] - self.__x[j])**2 + (self.__y[i] - self.__y[j])**2 < self.__params["c"]**2:
                j_list.append(j)
                if self.__memory[j]:
                    memory_j = self.__memory[j]
                    self.__receive[i](memory_j.curve, memory_j.group, memory_j.time_curve, memory_j.time, memory_j.pose2D_x, memory_j.pose2D_y, memory_j.pose2D_theta, memory_j.mov_will, memory_j.number,memory_j.state)
        return update_iResponse(j_list)

    def load_sim_param(self):
        # Load simulation parameters
        params = {
            "Rb": float(rospy.get_param("/Rb")),
            "d": float(rospy.get_param("/d")),
            "c": float(rospy.get_param("/c")),
            "ref_vel": float(rospy.get_param("/ref_vel"))
        }
        return params

    def callback_point(self,point,i):
        p = [point.x,point.y,0.5]
        self.cfs[self.__IDs[i]].cmdPosition(p, 0)

    def callback_pose(self,position,i):
        self.__start[i] = True
        self.__x[i] = position[0]
        self.__y[i] = position[1]

        p = Point()
        p.x = self.__x[i]
        p.y = self.__y[i]

        self.__pose_publisher[i].publish(p)

    def land(self):
        self.swarm.allcfs.land(targetHeight=0.0, duration=TAKEOFF_DURATION)
        self.timeHelper.sleep(TAKEOFF_DURATION)
        print("landed")
    
    def metric(self):
        S = [0 for i in range(self.__n_robots)]
        S_min = {g: np.inf for g in self.__group_list}
        S_max = {g: 0 for g in self.__group_list}
        for i in range(self.__n_robots):
            g = self.__groups[i]
            S[i] = round(np.sqrt(self.__x[i]**2 + self.__y[i]**2)/self.__params["d"])
            if not S:
                S = 1
            S_min[g] = min(S_min[g],S[i])
            S_max[g] = max(S_max[g],S[i])
        
        infringing = 0
        for i in range(self.__n_robots):
            for k in self.__group_list:
                if self.__groups[i] != k and S_min[k]<=S[i]<=S_max[k]:
                    infringing += 1
        
        segregation = 100*(1-infringing/(self.__n_robots*self.__n_groups))
        print(f"Segregation: {segregation}%")


if __name__ == "__main__":
    try:
        cf_yaml = f"/crazyswarm/ros_ws/src/ros_dubins_seg/launch/{sys.argv[1]}"
        n_robots = int(len(sys.argv)) - 4
        groups = []
        for aux in range(2, n_robots+2):
            groups.append(int(sys.argv[aux]))
        M_groups = max(groups)+1
        comm_node = CommNode(cf_yaml,groups,n_robots)
        comm_node.main_loop()
    except rospy.ROSInterruptException:
        pass
