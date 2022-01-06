#!/usr/bin/env python

import sys
import time

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from dubins_seg.srv import *


class CommNode():

    def __init__(self, n_robots, freq=20):
        self.__n_robots = n_robots
        self.__freq = float(freq)
        self.__x = [0]*self.__n_robots
        self.__y = [0]*self.__n_robots
        self.__theta = [0]*self.__n_robots
        self.__params = self.load_sim_param()      
        # Init node
        rospy.init_node('communication')
        # Topics
        for robot_number in range(self.__n_robots):
            rospy.Subscriber('robot_' + str(robot_number) + '/base_pose_ground_truth', Odometry, self.callback_pose, (robot_number))
        self.__rate = rospy.Rate(self.__freq)

    def main_loop(self):
        time.sleep(1)
        while not rospy.is_shutdown():
            self.__params = self.load_sim_param()
            for i in range(self.__n_robots):
                for j in range(i+1,self.__n_robots):
                    ij_communicate =  (self.__x[i] - self.__x[j])**2 + (self.__y[i] - self.__y[j])**2 < self.__params['c']**2
                    if ij_communicate:
                        try:
                            send_memory_i = rospy.ServiceProxy('/robot_' + str(i) + '/send_mem_' + str(i),send_memory)
                            send_memory_j = rospy.ServiceProxy('/robot_' + str(j) + '/send_mem_' + str(j),send_memory)
                            receive_memory_i = rospy.ServiceProxy('/robot_' + str(i) + '/' + 'receive_mem_' + str(i), receive_memory)
                            receive_memory_j = rospy.ServiceProxy('/robot_' + str(j) + '/' + 'receive_mem_' + str(j), receive_memory)
                            memory_i = send_memory_i(i,j)
                            memory_j = send_memory_j(j,i)
                            resp_i_recv_j = receive_memory_i(memory_j.curve_index, memory_j.group, memory_j.time_curve, memory_j.time, memory_j.pose2D_x, memory_j.pose2D_y, memory_j.pose2D_theta, memory_j.mov_will, memory_j.number)
                            resp_j_recv_i = receive_memory_j(memory_i.curve_index, memory_i.group, memory_i.time_curve, memory_i.time, memory_i.pose2D_x, memory_i.pose2D_y, memory_i.pose2D_theta, memory_i.mov_will, memory_i.number)
                        except:
                            pass

    def load_sim_param(self):
        # Load simulation parameters
        params = {
            'Rb': float(rospy.get_param('/Rb')),
            'd': float(rospy.get_param('/d')),
            'c': float(rospy.get_param('/c')),
            'ref_vel': float(rospy.get_param('/ref_vel'))
        }
        return params

    def callback_pose(self,data,robot_number):
        self.__x[robot_number] = data.pose.pose.position.x
        self.__y[robot_number] = data.pose.pose.position.y
        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
        self.__theta[robot_number] = euler[2]


if __name__ == '__main__':
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
