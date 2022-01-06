#!/usr/bin/env python

import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
# from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
# from visualization_msgs.msg import Marker, MarkerArray

from pydubinsseg.segregationcontrol import SegregationControl
from pydubinsseg import state


class ControlNode():

    def __init__(self, robot_number, robot_group, freq=10):
        self.__robot_number = robot_number
        self.__robot_group = robot_group
        self.__freq = float(freq)
        self.__segregation = SegregationControl(self.__robot_number, self.__robot_group, state['in circle'], self.load_sim_param())
        # Init node
        rospy.init_node( 'controller_' + str(self.__robot_number) )
        # Topics
        self.__publisher = rospy.Publisher('/robot_' + str(self.__robot_number) + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robot_' + str(self.__robot_number) + '/base_pose_ground_truth', Odometry, self.callback_pose)
        # rospy.Subscriber('/robot_' + str(self.__robot_number) + '/base_scan', LaserScan, self.callback_scan)
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.__rate = rospy.Rate(self.__freq)
        # # Services
        send_memory_obj = rospy.Service('send_mem_' + str(self.__robot_number), send_memory, self.send_memory)
        receive_memory_obj = rospy.Service('receive_mem_' + str(self.__robot_number), receive_memory, self.receive_memory)
        # Pub blank data to start
        self.pub_vel(0,0)

    def main_loop(self):
        time.sleep(0.5)
        self.__segregation.calculate_initial_conditions()
        time.sleep(0.5)
        while not rospy.is_shutdown():
            self.__segregation.set_params(self.load_sim_param())
            # Broadcast hi
            self.__segregation.update_memory_about_itself()
            if self.__segregation.get_state() == state['in circle']:
                # self.__segregation.recieve_j_memory(other_memory)
                self.__segregation.calculate_lap()
                self.__segregation.calculate_wills()
                # self.__segregation.prevent_collision()
                self.__segregation.evaluate_wills()   
            elif self.__segregation.get_state() == state['transition']:
                self.__segregation.check_arrival()
            [v,w] = self.__segregation.calculate_input_signals()
            self.pub_vel(v,w)

    def load_sim_param(self):
        # Load simulation parameters
        params = {
            'Rb': float(rospy.get_param('/Rb')),
            'd': float(rospy.get_param('/d')),
            'c': float(rospy.get_param('/c')),
            'ref_vel': float(rospy.get_param('/ref_vel'))
        }
        return params

    def pub_vel(self,v,w):
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        self.__publisher.publish(vel)
        self.__rate.sleep()

    def send_memory(self,req):
        curve_index_list = []
        group_list = []
        time_curve_list = []
        time_list = []
        pose2D_x_list = []
        pose2D_y_list = []
        pose2D_theta_list = []
        will_list = []
        number_list = []
        for item in self.__segregation.send_memory():
            curve_index_list.append( item['curve_index'] )
            group_list.append( item['group'] )
            time_curve_list.append( item['time_curve'] )
            time_list.append( item['time'] )
            pose2D_x_list.append( item['pose2D'][0] )
            pose2D_y_list.append( item['pose2D'][1] )
            pose2D_theta_list.append( item['pose2D'][2] )
            will_list.append( item['will'] )
            number_list.append( item['number'] )
        return send_memoryResponse(curve_index_list, group_list, time_curve_list, time_list, pose2D_x_list, pose2D_y_list, pose2D_theta_list, will_list, number_list)

    def receive_memory(self,req):
        j_memory_data_list = []
        n_items = len(req.curve_index)
        for index in range(n_items):
            item = {
                'group': req.group[index],
                'number': req.curve_index[index],
                'curve_index': req.curve_index[index],
                'time_curve': req.time_curve[index],
                'time': req.time[index],
                'pose2D': [req.pose2D_x[index], req.pose2D_y[index], req.pose2D_theta[index]],
                'will': req.mov_will[index]
            }
            j_memory_data_list.append(item)
        self.__segregation.recieve_memory(j_memory_data)
        return receive_memoryResponse(0,0)  

    def callback_pose(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        eul = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        theta = eul[2]
        pose2D = [x,y,theta]
        self.__segregation.set_pose2D(pose2D)

    # def callback_scan(self,data):
    #     angle_min = data.angle_min
    #     angle_max = data.angle_max
    #     angle_increment = data.angle_increment
    #     angle = [angle_min, angle_max, angle_increment]
    #     range_min = data.range_min
    #     range_max = data.range_max
    #     ranges = [range_min, range_max]
    #     range_sensor = RangeSensor(angle=angle, ranges=ranges)
    #     range_sensor.update_ranges(data.ranges, data.intensities)
    #     self.dubin.update_measurement(range_sensor)

    def callback_time(self, data):
        self.__segregation.set_time(float(data.clock.secs) + float(data.clock.nsecs)/1e9)


if __name__ == '__main__':
    try:
        robot_number = int(sys.argv[1]); robot_group  = int(sys.argv[2]); x_initial = float(sys.argv[3]); y_initial = float(sys.argv[4])
        control_node = ControlNode(robot_number, robot_group)
        control_node.main_loop()
    except rospy.ROSInterruptException:
        pass
    