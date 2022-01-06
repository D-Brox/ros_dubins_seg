#!/usr/bin/env python
import rospy
from math import pi,atan,atan2,sqrt,copysign,sin,cos,asin
from dubins_seg.srv import *

class VectorField():
    G = 1
    H = 0
    alpha = 0
    Fx = 0
    Fy = 0
    grad_alpha_x = 0
    grad_alpha_y = 0
    def set_alpha(self,alpha, grad_alpha_x, grad_alpha_y):
        self.alpha = alpha
        self.grad_alpha_x = grad_alpha_x
        self.grad_alpha_y = grad_alpha_y
 
    def calculate(self, KG=8.0, circulation_direction=1):
        self.G = -2*atan(KG*self.alpha)/pi
        self.H = copysign(1,circulation_direction)*sqrt(1-self.G**2+1e-6)
        norm_grad_alpha = sqrt(self.grad_alpha_x**2 + self.grad_alpha_y**2) 
        self.Fx = (self.G*self.grad_alpha_x - self.H*self.grad_alpha_y)/(norm_grad_alpha + 1e6)
        self.Fy = (self.G*self.grad_alpha_y + self.H*self.grad_alpha_x)/(norm_grad_alpha + 1e6)

class MovState():
    circle = 1
    transition = 0

class MovWill():
    no_will = 0
    inward = -1
    outward = 1

class MemoryItem():
    curve_index = 0
    group = 0
    time_curve = 0
    time = 0
    pose2D = [0,0,0]
    mov_will = MovWill.no_will
    number = 0

class RangeSensor():
    def __init__(self, angle=[0.0,0.0,0.0], ranges=[0.0,0.0]):
        self.angle_min = angle[0]
        self.angle_max = angle[1]
        self.angle_increment = angle[2]
        self.range_min = ranges[0]
        self.range_max = ranges[1]
        self.data_distancies = []
        self.data_intensities = []
    def update_ranges(self,distancies,intensities):
        self.data_distancies = distancies
        self.data_intensities = intensities
    def get_ranges(self):
        return self.data_distancies
    def get_ranges_with_angle(self):
        angle = self.angle_min
        ranges_with_angle = []
        for dist in self.data_distancies:
            angle += self.angle_increment
            if dist != self.range_max:
                ranges_with_angle.append( (dist,angle) )
        return ranges_with_angle


class Dubin():
    def __init__(self, number, group):
        self.number = number
        self.group = group
        self.memory_i = MemoryItem()
        self.memory_j = [MemoryItem()]

        self.x = 0
        self.y = 0
        self.theta = 0
        self.time = 0
        self.closest_circle = 1
        self.mov_radius = 1
        self.state = MovState.circle
        self.lap = False
        self.mov_will = MovWill.no_will
        self.curve_vector_field = VectorField()
        self.transition_vector_field = VectorField()
        self.theta_error = 0
        self.theta_error_int = 0
        self.theta_error_prev = 0
        self.v = 0
        self.w = 0
        self.transition_x = 0
        self.transition_y = 0
        self.transition_dir = 0
        self.range_sensor = RangeSensor()

    def get_number(self):
        return self.number

    def set_pose2D(self,pose2D):
        self.x = float(pose2D[0])
        self.y = float(pose2D[1])
        self.theta = float(pose2D[2])

    def get_pose2D(self):
        return [self.x, self.y, self.theta]

    def set_state(self,state):
        self.state = state

    def get_state(self):
        return self.state

    def calculate_curve_field(self):
        xp = self.x/abs(self.mov_radius)
        yp = self.y/abs(self.mov_radius)
        alpha = xp**2 + yp**2 - 1
        grad_alpha_x = 2*xp/abs(self.mov_radius)
        grad_alpha_y = 2*yp/abs(self.mov_radius)
        self.curve_vector_field.set_alpha(alpha, grad_alpha_x, grad_alpha_y)
        self.curve_vector_field.calculate(circulation_direction=self.mov_radius)  

    def calculate_transition_field(self,d):
        xp = (self.x - self.transition_x)/(d/2)
        yp = (self.y - self.transition_y)/(d/2)
        alpha = xp**2 + yp**2 - 1
        grad_alpha_x = 2*xp/(d/2)
        grad_alpha_y = 2*yp/(d/2)
        self.transition_vector_field.set_alpha(alpha, grad_alpha_x, grad_alpha_y)
        self.transition_vector_field.calculate(KG=5.0,circulation_direction=self.transition_dir)

    def set_v_w(self,v,delta_t,type, Kp=0.8, Ki=0.0, Kd=0.0):
        if type == 'curve':
            self.theta_ref = atan2(self.curve_vector_field.Fy, self.curve_vector_field.Fx )
        elif type == 'transition':
            self.theta_ref = atan2(self.transition_vector_field.Fy, self.transition_vector_field.Fx )
        else:
            msg = 'Robot ' + str(self.get_number()) + ',' + str(self.group) + ' undefined curve type for calculating v,w'
            rospy.logerr(msg) 
            self.theta_ref = self.theta
        self.theta_error = asin(sin(self.theta_ref - self.theta))

        self.theta_error_int = self.theta_error_int + self.theta_error
        self.theta_error_diff = (self.theta_error - self.theta_error_prev)/delta_t
        self.theta_error_prev = self.theta_error

        self.w = Kp*self.theta_error + Ki*self.theta_error_int + Kd*self.theta_error_diff
        self.v = v

    def get_v_w(self):
        return self.v, self.w

    def calculate_lap(self):
        error_limit = 0.01
        time_per_curve = 60
        theta_diff = abs(self.theta_curve - self.theta)
        condition_theta = theta_diff <= error_limit or theta_diff >= 2*pi-error_limit
        condition_time = self.time/1000 - self.time_curve/1000 > time_per_curve*self.closest_circle 
        if condition_theta and condition_time:
            self.set_lap(True)

    def set_lap(self, lap):
        self.lap = lap
        self.time_curve = self.time
        self.theta_curve = self.theta

    def calculate_circle(self,d):
        self.closest_circle = max(round(sqrt(self.x**2 + self.y**2)/d), 1)
        self.mov_radius = self.closest_circle*d*((-1)**(self.closest_circle))

    def reset_memory(self):
        self.set_lap(False)
        self.set_state(MovState.circle)
        self.mov_will = MovWill.no_will
        robot_i = MemoryItem()
        robot_i.curve_index = self.closest_circle
        robot_i.group = self.group
        robot_i.time_curve = self.time
        robot_i.time = self.time
        robot_i.pose2D = self.get_pose2D()
        robot_i.mov_will = self.mov_will
        robot_i.number = self.get_number()
        self.memory_i = robot_i
        self.memory_j = []

    def send_memory(self,req):
        curve_index_list = [self.memory_i.curve_index]
        group_list = [self.memory_i.group]
        time_curve_list = [self.memory_i.time_curve]
        time_list = [self.memory_i.time]
        pose2D_x_list = [self.memory_i.pose2D[0]]
        pose2D_y_list = [self.memory_i.pose2D[1]]
        pose2D_theta_list = [self.memory_i.pose2D[2]]
        mov_will_list = [self.memory_i.mov_will]
        number_list = [self.memory_i.number]
        for robot_j in self.memory_j:
            curve_index_list.append(robot_j.curve_index)
            group_list.append(robot_j.group)
            time_curve_list.append(robot_j.time_curve)
            time_list.append(robot_j.time)
            pose2D_x_list.append(robot_j.pose2D[0])
            pose2D_y_list.append(robot_j.pose2D[1])
            pose2D_theta_list.append(robot_j.pose2D[2])
            mov_will_list.append(robot_j.mov_will)
            number_list.append(robot_j.number)
        return send_memoryResponse(curve_index_list, group_list, time_curve_list, time_list, pose2D_x_list, pose2D_y_list, pose2D_theta_list, mov_will_list, number_list)

    def receive_memory(self,req):
        N_items = len(req.curve_index)
        number_list = []
        for robot_j in self.memory_j:
            number_list.append(robot_j.number)
        for j in range(N_items):
            if req.number[j] == self.get_number():
                continue
            if req.number[j] in number_list:
                j_index = number_list.index(req.number[j])
                if req.time[j] > self.memory_j[j_index].time:
                    update_robot_j = MemoryItem()
                    update_robot_j.curve_index = req.curve_index[j]
                    update_robot_j.group = req.group[j]
                    update_robot_j.time_curve = req.time_curve[j]
                    update_robot_j.time = req.time[j]
                    update_robot_j.pose2D[0] = req.pose2D_x[j]
                    update_robot_j.pose2D[1] = req.pose2D_y[j]
                    update_robot_j.pose2D[2] = req.pose2D_theta[j]
                    update_robot_j.mov_will = req.mov_will[j]
                    update_robot_j.number = req.number[j]
                    self.memory_j[j_index] = update_robot_j
                else:
                    pass
            else:
                new_robot_j = MemoryItem()
                new_robot_j.curve_index = req.curve_index[j]
                new_robot_j.group = req.group[j]
                new_robot_j.time_curve = req.time_curve[j]
                new_robot_j.time = req.time[j]
                new_robot_j.pose2D[0] = req.pose2D_x[j]
                new_robot_j.pose2D[1] = req.pose2D_y[j]
                new_robot_j.pose2D[2] = req.pose2D_theta[j]
                new_robot_j.mov_will = req.mov_will[j]
                new_robot_j.number = req.number[j]
                self.memory_j.append(new_robot_j)
        
        return receive_memoryResponse(0,0)  

    def update_measurement(self,measurement):
        self.range_sensor = measurement

    def get_obstacles_position(self):
        obstacles_position = []
        for dist,angle in self.range_sensor.get_ranges_with_angle():
            xo = self.x + dist*cos(self.theta + angle)
            yo = self.y + dist*sin(self.theta + angle)
            obstacles_position.append( (xo,yo) )
            # msg = 'Robot ' + str(self.get_number()) + ',' + str(self.group) + ' Measurement: ' + str( [(dist,angle),(xo,yo),(self.x, self.y, self.theta)] ) 
            # rospy.loginfo(msg)
        return obstacles_position

    def calculate_wills(self):
        robot_i = self.memory_i
        inward = False
        outward = False
        for robot_j in self.memory_j:
            j_is_inward = robot_j.curve_index < robot_i.curve_index
            j_is_immediate_inward = robot_j.curve_index == robot_i.curve_index - 1
            j_is_same_curve = robot_j.curve_index == robot_i.curve_index
            j_is_same_group = robot_j.group == robot_i.group
            j_wants_outward = robot_j.mov_will == MovWill.outward
            j_arrived_first = robot_j.time_curve < robot_i.time_curve
            j_tiebreaker = (robot_j.time_curve == robot_i.time_curve) and robot_j.pose2D[2] < robot_i.pose2D[2]
            for robot_k in self.memory_j:
                j_group_is_alone = True
                k_is_same_group_j = robot_k.group == robot_j.group
                k_is_same_curve_j = robot_k.curve_index == robot_j.curve_index
                if k_is_same_curve_j and not k_is_same_group_j:
                    j_group_is_alone = False
                    break

            if j_is_immediate_inward and not j_is_same_group:
                self.set_lap(False)
            if j_is_same_group and j_group_is_alone and j_is_inward:
                inward = True
            if j_is_same_curve and not j_is_same_group and not inward:
                if j_arrived_first or j_tiebreaker:
                    outward  = True
            if j_is_immediate_inward and j_wants_outward:
                outward = True

        if self.lap and robot_i.curve_index > 1:
            inward = True

        if outward:
            self.mov_will = MovWill.outward
        elif inward:
            self.mov_will = MovWill.inward
        else:
            #self.mov_will = MovWill.no_will
            pass

        #msg = 'Robot ' + str(self.get_number()) + ',' + str(self.group) + ' will = ' + str(self.mov_will)
        #rospy.loginfo(msg)

    def evaluate_wills(self,d):
        if self.mov_will == MovWill.outward:
            self.set_state(MovState.transition)
            self.set_lap(False)
            self.closest_circle = self.closest_circle + 1
            self.mov_radius = self.closest_circle*d*((-1)**(self.closest_circle))
            msg = 'Robot ' + str(self.get_number()) + ',' + str(self.group) + ' moving OUTWARDS, state = ' + str(self.get_state()) + ', radius = ' + str(self.mov_radius)
            rospy.loginfo(msg)
        elif self.mov_will == MovWill.inward:
            self.set_state(MovState.transition)
            self.set_lap(False)
            self.closest_circle = max(self.closest_circle - 1, 1)
            self.mov_radius = self.closest_circle*d*((-1)**(self.closest_circle))
            msg = 'Robot ' + str(self.get_number()) + ',' + str(self.group) + ' moving INWARDS, state = ' + str(self.get_state()) + ', radius = ' + str(self.mov_radius)
            rospy.loginfo(msg)
        else:
            pass

    def calculate_transition_curve(self,d):
        if self.mov_will == MovWill.outward:
            self.transition_x = self.x*(1 + d/(2*sqrt(self.x**2 + self.y**2)))
            self.transition_y = self.y*(1 + d/(2*sqrt(self.x**2 + self.y**2)))
            self.transition_dir = copysign(1,self.mov_radius)
        elif self.mov_will == MovWill.inward:
            self.transition_x = self.x*(1 - d/(2*sqrt(self.x**2 + self.y**2)))
            self.transition_y = self.y*(1 - d/(2*sqrt(self.x**2 + self.y**2)))
            self.transition_dir = copysign(1,-self.mov_radius)
        else:
            pass

    def prevent_collision(self,d,in_a_curve_epsilon_percentage = 0.1):
        if self.mov_will != MovWill.no_will or True:
            for xo,yo in self.get_obstacles_position():
                diff_from_curve = sqrt(xo**2 + yo**2)%d
                if diff_from_curve >= (1-in_a_curve_epsilon_percentage)*d:
                    j_curve_index = sqrt(xo**2 + yo**2)//d + 1
                    obst_in_curve = True
                elif diff_from_curve <= in_a_curve_epsilon_percentage*d:
                    j_curve_index = sqrt(xo**2 + yo**2)//d
                    obst_in_curve = True
                else:
                    j_curve_index = sqrt(xo**2 + yo**2)//d + 1
                    obst_in_curve = False 
                if not obst_in_curve:
                    self.mov_will = MovWill.no_will
                    # msg = 'Robot ' + str(self.get_number()) + ',' + str(self.group) + ' Prevented! ' + str( [(xo,yo),diff_from_curve,j_curve_index,obst_in_curve] ) 
                    # rospy.loginfo(msg)
                    break

# Unit test
if __name__ == '__main__':
    # N/A
	pass