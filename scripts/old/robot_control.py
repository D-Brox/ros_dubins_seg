#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from math import sqrt, atan2, exp, log, atan, cos, sin, acos, pi, asin, atan2, copysign
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
import sys
from dubins_seg.srv import *
import numpy as np

from Dubin import *

class ControlNode():

    def __init__(self, dubin, freq=10):
        self.dubin = dubin
        self.freq = float(freq)
        
        # Init node
        rospy.init_node('controller_n' + str(self.dubin.get_number()) )
        # Topics
        self.publisher = rospy.Publisher('/robot_' + str(self.dubin.get_number()) + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robot_' + str(self.dubin.get_number()) + '/base_pose_ground_truth', Odometry, self.callback_pose)
        rospy.Subscriber('/robot_' + str(self.dubin.get_number()) + '/base_scan', LaserScan, self.callback_scan)
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.rate = rospy.Rate(self.freq)
        # Services
        send_memory_obj = rospy.Service('send_mem_' + str(self.dubin.get_number()), send_memory, self.dubin.send_memory)
        receive_memory_obj = rospy.Service('receive_mem_' + str(self.dubin.get_number()), receive_memory, self.dubin.receive_memory)
        # Pub blank data to start
        self.pub_vel(0,0)

    def main_loop(self):
        while not rospy.is_shutdown():
            self.load_sim_param()
            # Broadcast hi
            # Update memory about himself # maybe its better after
            if self.dubin.get_state() == MovState.circle:
                # Update memory about others
                self.dubin.calculate_lap()

                self.dubin.calculate_wills()
                self.dubin.prevent_collision(self.d)
                self.dubin.evaluate_wills(self.d)
                self.dubin.calculate_transition_curve(self.d)

                self.dubin.calculate_curve_field()
                self.dubin.set_v_w(self.ref_vel,1.0/self.freq,'curve')
            elif self.dubin.get_state() == MovState.transition:
                self.dubin.calculate_curve_field()
                self.dubin.calculate_transition_field(self.d)
                self.dubin.set_v_w(self.ref_vel,1.0/self.freq,'transition')
                if self.dubin.curve_vector_field.G**2 < 0.19**2:
                    self.dubin.reset_memory()
                    msg = 'Robot ' + str(self.dubin.get_number()) + ',' + str(self.dubin.group) + ' ARRIVED'
                    rospy.loginfo(msg)
            else:
                rospy.logerr('Undefined robot', str(self.dubin.get_number()), 'state', str(self.dubin.get_state()))

            [v,w] = self.dubin.get_v_w()
            self.pub_vel(v,w)

    def load_sim_param(self):
        # Load simulation parameters
        self.Rb = float(rospy.get_param('/Rb'))
        self.d = float(rospy.get_param('/d'))
        self.c = float(rospy.get_param('/c'))
        self.ref_vel = float(rospy.get_param('/ref_vel'))

    def init_dubin(self,wait_time_init=15):
        self.load_sim_param()
        self.dubin.set_state(MovState.circle)
        self.dubin.set_lap(False)
        self.dubin.calculate_circle(self.d)
        self.dubin.reset_memory()
        self.pub_vel(0,0)
        while self.dubin.time/1000 <= wait_time_init:
            self.load_sim_param()
            self.dubin.set_state(MovState.circle)
            self.dubin.set_lap(False)
            self.dubin.calculate_circle(self.d)
            self.dubin.reset_memory()
            self.pub_vel(0,0)
        debug_msg = 'Robot ' + str(self.dubin.get_number()) + ' initialized.' + ' State: ' + str(self.dubin.state) + ' r: ' + str(self.dubin.mov_radius)
        rospy.loginfo(debug_msg)

    def pub_vel(self,v,w):
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        self.publisher.publish(vel)
        self.rate.sleep()

    def callback_pose(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        eul = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        theta = eul[2]
        pose2D = [x,y,theta]
        self.dubin.set_pose2D(pose2D)

    def callback_scan(self,data):
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        angle = [angle_min, angle_max, angle_increment]
        range_min = data.range_min
        range_max = data.range_max
        ranges = [range_min, range_max]
        range_sensor = RangeSensor(angle=angle, ranges=ranges)
        range_sensor.update_ranges(data.ranges, data.intensities)
        self.dubin.update_measurement(range_sensor)

    def callback_time(self, data):
        self.time = data.clock.secs*1e3 + data.clock.nsecs/1e6
        self.dubin.time = self.time

if __name__ == '__main__':
    try:
        robot_number = int(sys.argv[1]); robot_group  = int(sys.argv[2]); x_initial = float(sys.argv[3]); y_initial = float(sys.argv[4])
        dubin = Dubin(robot_number,robot_group)
        control_node = ControlNode(dubin)
        control_node.init_dubin()
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass