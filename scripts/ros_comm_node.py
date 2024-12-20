#!/usr/bin/env python

import sys
import time
from collections import Counter
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from tf.transformations import euler_from_quaternion

from pydubinsseg import movement_will

from dubins_seg.srv import *

class CommNode():

    def __init__(self, n_robots, freq=1000):
        self.__n_robots = n_robots
        self.__freq = float(freq)
        self.__x = [0]*self.__n_robots
        self.__y = [0]*self.__n_robots
        self.__theta = [0]*self.__n_robots
        self.__memory = [None]*self.__n_robots
        self.__groups = groups
        self.__group_list = list(set(groups))
        self.__n_groups = len(self.__group_list)
        self.__segregated_groups = set()

        self.__params = self.load_sim_param()
        self.__start = [False for _ in range(self.__n_robots)]
        # Init node
        rospy.init_node("communication")
        # Topics
        self.__send = [None]*self.__n_robots
        self.__receive = [None]*self.__n_robots
        self.__get_will = [None]*self.__n_robots
        for i in range(self.__n_robots):
            rospy.wait_for_service(f"/robot_{i}/send_mem_{i}")
            rospy.wait_for_service(f"/robot_{i}/receive_mem_{i}")
            self.__send[i] = rospy.ServiceProxy(f"/robot_{i}/send_mem_{i}",send_memory,persistent=True)
            self.__receive[i] = rospy.ServiceProxy(f"/robot_{i}/receive_mem_{i}", receive_memory,persistent=True)
            rospy.Subscriber(f"robot_{i}/base_pose_ground_truth", Odometry, self.callback_pose, (i))
            self.__get_will[i] = rospy.Publisher(f"robot_{i}/will",Int64,queue_size=10)

        rospy.Service("start",start,lambda _:all(self.__start) and all(self.__start))
        rospy.Service("update_i",update_i,self.update_i)

        self.__rate = rospy.Rate(self.__freq)

    def main_loop(self):
        self.__params = self.load_sim_param()

        while not rospy.is_shutdown():
            self.receive_all()
            self.calculate_wills()

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

    def callback_pose(self,data,robot_number):
        self.__start[robot_number] = True
        self.__x[robot_number] = data.pose.pose.position.x
        self.__y[robot_number] = data.pose.pose.position.y
        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
        self.__theta[robot_number] = euler[2]

    def calculate_wills(self):
        (S, S_min, S_max, S_mean) = self.S_min_max_mean()
        S_count = Counter(S)
        S_mean = [(g,m) for g,m in S_mean.items()]
        S_mean.sort(key= lambda x: x[1])
        a = {k:0 for k in self.__group_list}
        for i in range(self.__n_robots):
            for k in self.__group_list:
                if self.__groups[i] != k and S_min[k]<=S[i]<=S_max[k]:
                    a[k] += 1
        
        will = [movement_will["none"]]*self.__n_robots
        S_prev_max = 0
        g_next = None
        for (g,_) in S_mean:
            if a[g] == 0:
                for i in range(self.__n_robots):
                    # Use the least space possible
                    if self.__groups[i]==g and S[i]>S_prev_max+1:
                        will[i] = movement_will["inward"]
                self.__segregated_groups.add(g)
                S_prev_max = S_max[g]
            else:
                g_next = g
                break
        if g_next is None:
            S_next_max = S_prev_max+1
        else:                  
            S_next_max = S_max[g_next]
        for i in range(self.__n_robots):
            if self.__groups[i] in self.__segregated_groups:
                continue
            elif self.__groups[i] == g_next:
                if S[i]>S_prev_max+1:
                    will[i] = movement_will["inward"]
            elif S[i] <= S_next_max:
                will[i] = movement_will["outward"]
            # TODO: Implement occupancy:
            # elif not has_space(S[i],S_count[S[i]]):
            #     will[i] = movement_will["outwards"]
        for i in range(self.__n_robots):
            self.__get_will[i].publish(will[i])

    def S_min_max_mean(self):
        S = [0 for i in range(self.__n_robots)]
        S_min = {g: np.inf for g in self.__group_list}
        S_max = {g: 0 for g in self.__group_list}
        S_mean = {g:0 for g in self.__group_list}
        n_k = {g: 0 for g in self.__group_list}
        for i in range(self.__n_robots):
            g = self.__groups[i]
            n_k[g] += 1
            S[i] = round(np.sqrt(self.__x[i]**2 + self.__y[i]**2)/self.__params["d"])
            if not S:
                S = 1
            S_min[g] = min(S_min[g],S[i])
            S_max[g] = max(S_max[g],S[i])
            S_mean[g] += S[i]
        
        for g in self.__group_list:
            S_mean[g] /= n_k[g]
        
        return S,S_min,S_max,S_mean

    def metric(self):
        (S, S_min, S_max, _) = self.S_min_max_mean()
        infringing = 0
        for i in range(self.__n_robots):
            for k in self.__group_list:
                if self.__groups[i] != k and S_min[k]<=S[i]<=S_max[k]:
                    infringing += 1
        
        segregation = 100*(1-infringing/(self.__n_robots*self.__n_groups))
        print(f"Segregation: {segregation}%")

if __name__ == "__main__":
    try:
        n_robots = int(len(sys.argv)) - 3
        groups = []
        for aux in range(1, n_robots+1):
            groups.append(int(sys.argv[aux]))
        M_groups = max(groups)+1
        comm_node = CommNode(n_robots)
        comm_node.main_loop()
    except rospy.ROSInterruptException:
        pass
