#!/usr/bin/env python

import sys
import time
from itertools import combinations

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

from dubins_seg.srv import *

class CommNode():

    def __init__(self, n_robots, freq=1000):
        self.__n_robots = n_robots
        self.__freq = float(freq)
        self.__x = [0]*self.__n_robots
        self.__y = [0]*self.__n_robots
        self.__theta = [0]*self.__n_robots
        self.__memory = [None]*self.__n_robots
        self.__params = self.load_sim_param()
        self.__start = [False for _ in range(self.__n_robots)]
        # Init node
        rospy.init_node("communication")
        # Topics
        for robot_number in range(self.__n_robots):
            rospy.Subscriber(f"robot_{robot_number}/base_pose_ground_truth", Odometry, self.callback_pose, (robot_number))
        rospy.Service("start",start,lambda _:all(self.__start) and all(self.__start))

        self.__send = [rospy.ServiceProxy(f"/robot_{i}/send_mem_{i}",send_memory,persistent=True) for i in range(self.__n_robots)]
        self.__receive = [rospy.ServiceProxy(f"/robot_{i}/receive_mem_{i}", receive_memory,persistent=True) for i in range(self.__n_robots)]
        rospy.Service("update_i",update_i,self.update_i)

        self.__rate = rospy.Rate(self.__freq)

    def main_loop(self):
        self.__params = self.load_sim_param()

        while not rospy.is_shutdown():
            self.receive_all()
            self.__rate.sleep()

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


if __name__ == "__main__":
    try:
        time.sleep(5)

        n_robots = int(len(sys.argv)) - 3
        groups = []
        for aux in range(1, n_robots+1):
            groups.append(int(sys.argv[aux]))
        M_groups = max(groups)+1
        comm_node = CommNode(n_robots)
        comm_node.main_loop()
    except rospy.ROSInterruptException:
        pass
