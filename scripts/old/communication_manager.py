#!/usr/bin/env python
from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import tf
from dubins_seg.srv import *

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class CommunicationManager():
    def __init__(self, N_robots, groups, M_groups,freq=20):
        self.N_robots = N_robots
        self.freq = float(freq)

        self.Rb = 0
        self.d = 0
        self.c = 0
        self.ref_vel = 0
        self.x = [0]*self.N_robots
        self.y = [0]*self.N_robots
        self.theta = [0]*self.N_robots
        
        # Init node
        rospy.init_node('communication_manager')
        # Topics
        for robot_number in range(self.N_robots):
            msg = 'Subscribing to ' + 'robot_' + str(robot_number) + '/base_pose_ground_truth'
            rospy.loginfo(msg)
            rospy.Subscriber('robot_' + str(robot_number) + '/base_pose_ground_truth', Odometry, self.callback_pose, (robot_number))
        self.rate = rospy.Rate(self.freq)

        

    def main_loop(self):
        while not rospy.is_shutdown():
            self.load_sim_param()
            for i in range(self.N_robots):
                for j in range(i+1,self.N_robots):
                    j_is_in_c_radius = (self.x[i] - self.x[j])**2 + (self.y[i] - self.y[j])**2 < self.c**2
                    if j_is_in_c_radius:
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
        self.Rb = float(rospy.get_param('/Rb'))
        self.d = float(rospy.get_param('/d'))
        self.c = float(rospy.get_param('/c'))
        self.ref_vel = float(rospy.get_param('/ref_vel'))

    def callback_pose(self,data,robot_number):
        self.x[robot_number] = data.pose.pose.position.x
        self.y[robot_number] = data.pose.pose.position.y
        x_q = data.pose.pose.orientation.x
        y_q = data.pose.pose.orientation.y
        z_q = data.pose.pose.orientation.z
        w_q = data.pose.pose.orientation.w
        euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
        self.theta[robot_number] = euler[2]

if __name__ == '__main__':
    try:
        N_robots = int(len(sys.argv)) - 3
        groups = []
        for aux in range(1, N_robots+1):
            groups.append(int(sys.argv[aux]))
        M_groups = max(groups)+1

        communication_manager = CommunicationManager(N_robots, groups, M_groups)

        communication_manager.main_loop()
    except rospy.ROSInterruptException:
        pass
