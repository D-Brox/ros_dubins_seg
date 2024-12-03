#!/usr/bin/env python

import sys
import time
from itertools import combinations
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler

from pycrazyswarm import Crazyswarm

from dubins_seg.srv import *

TAKEOFF_DURATION = 5.0

class CommNode():

    def __init__(self, groups,n_robots, freq=60):
        self.swarm = Crazyswarm(crazyflies_yaml="/crazyswarm/ros_ws/src/ros_dubins_seg/launch/dubins05.yaml",args="--sim --vis mpl")
        self.timeHelper = self.swarm.timeHelper
        self.cfs = self.swarm.allcfs.crazyfliesById
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
        self.__publisher = [None]*self.__n_robots
        for robot_number in range(self.__n_robots):
            self.cfs[robot_number+1].setLEDColor(*({0:[1,0,0],1:[0,1,0],2:[0,0,1]}[groups[robot_number]]))
            self.__publisher[robot_number] = rospy.Publisher(f"robot_{robot_number}/base_pose_ground_truth", Odometry, queue_size=10)
            rospy.Subscriber(f"/robot_{robot_number}/cmd_vel", Twist, self.callback_vel,(robot_number))
        rospy.Service("start",start,lambda _:all(self.__start) and all(self.__start))

        self.__send = [rospy.ServiceProxy(f"/robot_{i}/send_mem_{i}",send_memory,persistent=True) for i in range(self.__n_robots)]
        self.__receive = [rospy.ServiceProxy(f"/robot_{i}/receive_mem_{i}", receive_memory,persistent=True) for i in range(self.__n_robots)]
        rospy.Service("update_i",update_i,self.update_i)

        self.__rate = rospy.Rate(self.__freq)

    def main_loop(self):
        self.__params = self.load_sim_param()

        # Takeoff
        self.swarm.allcfs.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
        self.timeHelper.sleep(TAKEOFF_DURATION)

        while not rospy.is_shutdown():
            for i,ID in enumerate(range(1,6)):
                position = self.cfs[ID].position()
                yaw = self.cfs[ID].yaw()
                self.callback_pose((position,yaw),i)
            self.receive_all()
            self.swarm.timeHelper.sleep(1/self.__freq)
            self.__rate.sleep()

    def receive_all(self):
        for i in range(self.__n_robots):
            self.__memory[i] = self.__send[i](i,i)
            # print(i,self.__memory[i].mov_will)

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

    def callback_vel(self,twist,robot_number):
        vel = [twist.linear.x,twist.linear.y,twist.linear.z]
        yaw = twist.angular.z
        self.cfs[robot_number+1].cmdVelocityWorld(vel, yaw*180/np.pi)

    def callback_pose(self,pose,robot_number):
        (position,yaw) = pose
        self.__start[robot_number] = True
        self.__x[robot_number] = position[0]
        self.__y[robot_number] = position[1]
        self.__theta[robot_number] = yaw
        # if not robot_number:
            # print(yaw)

        odom = Odometry()
        odom.pose.pose.position.x = self.__x[robot_number]
        odom.pose.pose.position.y = self.__y[robot_number]

        quaternion = quaternion_from_euler(0.0,0.0,self.__theta[robot_number])
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        self.__publisher[robot_number].publish(odom)

if __name__ == "__main__":
    try:
        n_robots = int(len(sys.argv)) - 3
        groups = []
        for aux in range(1, n_robots+1):
            groups.append(int(sys.argv[aux]))
        M_groups = max(groups)+1
        comm_node = CommNode(groups,n_robots)
        comm_node.main_loop()
    except rospy.ROSInterruptException:
        pass
