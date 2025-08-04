#!/usr/bin/env python
import numpy as np
from numpy import pi
from numpy import sqrt

from vector_field import CircleVectorField
from vector_utils import ang_vec, ang_vec_diff, ang_diff, targ_diff, dot_vec
from state import MovementWill, State

class SegregationController():

    def __init__(self):
        self.memory = None
        self.vector_field = CircleVectorField(1,0,0)
        self.lap = False

    def init_vector_field(self):
        current_curve = self.memory.data[0].curve
        r = current_curve*self.memory.d
        dir = ((-1)**(current_curve))
        self.vector_field.redefine(r,0,0,dir=dir)

    def calculate_lap(self):
        error_limit = 0.01
        time_per_curve = 1.8 * np.pi * self.memory.d / self.memory.v # close to completing
        theta_diff = abs(self.memory.theta_curve - self.memory.data[0].pose[2])
        condition_theta = theta_diff <= error_limit or theta_diff >= 2*pi-error_limit
        condition_time = self.memory.data[0].time - self.memory.data[0].time_curve > time_per_curve*self.memory.data[0].curve
        if condition_theta and condition_time:
            self.set_lap(True)

    def set_lap(self, lap):
        self.lap = lap

    def check_memory_lap(self):
        if self.lap != self.memory.lap:
            self.memory.lap = self.lap
            memory = self.memory.data[0]
            memory.time_curve = self.memory.data[0].time
            self.memory.theta_curve = memory.pose[2]
            self.memory.data[0] = memory

    def set_state(self,state):
        self.memory.data[0].state = state

    def compute_next(self,dt):
        return self.vector_field.compute_next(self.memory.data[0].pose,self.memory.v,dt)

    
    def calculate_lower_control(self):
        pose = self.memory.data[0].pose
        [F,_,_,_,_,delta] = self.vector_field.compute_field(pose)
        Kp = 50
        theta_ref = np.atan2(F[1], F[0])
        theta_error = np.asin(np.sin(theta_ref - pose[2]))
        w = Kp*theta_error
        v =  self.memory.v * delta
        return (v,w)

    def calculate_will(self):
        i_data = self.memory.itself()

        make_room = False
        inward = False
        outward = False

        neighbors_same_curve = []
        k_data_same_group_inside = []
        k_other_group_same_curve = []
        for j_data in self.memory.neighborhood():
            # A1: l13-l15
            if j_data.group != i_data.group and j_data.curve == i_data.curve - 1:
                self.set_lap(False)

            if j_data.curve == i_data.curve:
                neighbors_same_curve.append(j_data)

        for k_data in self.memory.others():
            if k_data.group != i_data.group:
                if k_data.curve == i_data.curve:
                    k_other_group_same_curve.append(k_data)
            elif k_data.curve < i_data.curve:
                    k_data_same_group_inside.append(k_data)
            if k_data.group != i_data.group and k_data.will == MovementWill.Outward.value and k_data.curve == (i_data.curve - 1):
                make_room = True

        k_same_group_alone_inside = False
        if k_data_same_group_inside:
            k_same_group_alone_inside = True
            for k_data in k_data_same_group_inside:
                for l_data in self.memory.others():
                    if not l_data.state:
                        continue
                    if k_data.curve != l_data.curve:
                        continue
                    if k_data.group == l_data.group:
                        continue
                    break
                else: # only executed if the inner loop did NOT break
                    continue
                k_same_group_alone_inside = False
                break

        #A1: l22-l23
        if make_room:
            close = []
            ang_Sr = self.security_distance(i_data.curve)/(i_data.curve * self.memory.d)
            for j_data in neighbors_same_curve:
                if ang_vec_diff(i_data.pose,j_data.pose) < 1.5 * ang_Sr:
                    close.append(j_data)
            if len(close) >= 2:
                outward = True

        #A1: l17-l16
        if k_same_group_alone_inside:
            inward = True
        #A1: l19-l21
        if k_other_group_same_curve and not inward:
            outward  = False
            for k_data in k_other_group_same_curve:
                if k_data.state and (i_data.time_curve > k_data.time_curve or (i_data.time_curve == k_data.time_curve and (i_data.pose[2] > k_data.pose[2]))):
                    outward |= True
                    break

        #A1: l22-l23
        if self.memory.lap and i_data.curve > 1:
            inward = True

        if outward:
            i_data.will = MovementWill.Outward.value
        elif inward:
            i_data.will = MovementWill.Inward.value
        else:
            i_data.will = MovementWill.Stay.value

        self.memory.data[0] = i_data
        return inward,outward

    def security_distance(self,curve):
        d = (curve-0.5)*self.memory.d
        r1 = curve*self.memory.d -  self.memory.Rb*2
        r2 = self.memory.d/2
        l = (r2**2 - r1**2 + d**2)/(2*d)
        return (pi-np.arccos(l/r2))*r2

    def tunnel_angles(self, curr, targ):
        d = (targ+curr)/2*self.memory.d
        r1 = targ*self.memory.d
        r2 = self.memory.d/2 + self.memory.Rb*2
        l = (r1**2 - r2**2 + d**2)/(2*d)

        r = self.memory.d/2
        goal = pi*r
        ang_goal = goal/r1

        Sr = self.security_distance(targ)
        ang_Sr = Sr/r1

        ang_Pi = 2*np.arcsin(self.memory.Rb/r2)*r/r1
        ang_Pj = np.arccos(l/r1)
        return ang_Sr,ang_goal,ang_Pi,ang_Pj

    def prevent_collision(self,inward,outward):
        i_data = self.memory.itself()
        tunnel_in = []
        tunnel_out = []
        preferencial = True
        for j_data in self.memory.neighborhood():
            if abs(i_data.curve - j_data.curve) > 2:
                continue

            if not j_data.state:
                preferencial = False

            #A2: l3-l5
            if inward:
                if i_data.curve - 1 == j_data.curve:
                    tunnel_in.append(j_data)

            #A2: l6-l8
            if outward:
                if i_data.curve + 1 == j_data.curve:
                    tunnel_out.append(j_data)

            if j_data.will != MovementWill.Stay.value and ang_vec(j_data.pose) < ang_vec(i_data.pose):
                if j_data.curve == i_data.curve:
                    preferencial = False
                else:
                    if outward and j_data.curve > i_data.curve:
                        preferencial = False
                    if inward and j_data.curve < i_data.curve:
                        preferencial = False

        if inward:
            targ = i_data.curve - 1
            ang_Sr,ang_goal,ang_Pi, ang_Pj = self.tunnel_angles(i_data.curve,targ)
            for j_data in tunnel_in:
                if j_data.group == i_data.group and targ != 1:
                    ang_Sr_l=2*ang_Sr
                else:
                    ang_Sr_l = ang_Sr

                ang_before = ang_goal - ang_Pi - ang_Sr_l
                ang_after = ang_goal + ang_Pj + ang_Sr_l
                ang_diff = targ_diff(i_data.pose,j_data.pose,targ)
                if ang_before < ang_diff < ang_after and dot_vec(i_data.pose,j_data.pose)>=0:
                    inward = False
                    break

        if outward:
            targ = i_data.curve + 1
            ang_Sr,ang_goal,ang_Pi, ang_Pj = self.tunnel_angles(i_data.curve,targ)
            for j_data in tunnel_out:
                if j_data.group == i_data.group:
                    ang_Sr_l=2*ang_Sr
                else:
                    ang_Sr_l = ang_Sr

                ang_before = ang_goal - ang_Pi - ang_Sr_l
                ang_after = ang_goal + ang_Pj + ang_Sr_l
                ang_diff = targ_diff(i_data.pose,j_data.pose,targ)
                if ang_before < ang_diff < ang_after and dot_vec(i_data.pose,j_data.pose)>=0:
                    outward = False
                    break

        if outward:
            i_data.will = MovementWill.Outward.value
            if preferencial:
                self.evaluate_transition_field()
        elif inward:
            i_data.will = MovementWill.Inward.value
            if preferencial:
                self.evaluate_transition_field()
        self.memory.data[0] = i_data

    def evaluate_transition_field(self):
        self.set_state(State.Transition.value)
        self.set_lap(False)
        p = self.memory.data[0].pose
        self.memory.theta_curve = p[2]
        r = self.memory.d/2
        sig = 1 if self.memory.data[0].will == MovementWill.Outward.value else -1
        cx = p[0]*(sqrt(p[0]**2 + p[1]**2)+sig*r)/sqrt(p[0]**2 + p[1]**2)
        cy = p[1]*(sqrt(p[0]**2 + p[1]**2)+sig*r)/sqrt(p[0]**2 + p[1]**2)
        self.memory.desired_curve = max(self.memory.data[0].curve+sig,1)
        dir = ((-1)**(self.memory.desired_curve)) if sig == 1 else None
        self.vector_field.redefine(r, cx, cy, dir = dir)

    def check_arrival(self, tol = 0.15):
        r = self.memory.desired_curve*self.memory.d
        memory = self.memory.data[0]
        theta = memory.pose[2]
        theta_start = self.memory.theta_curve
        if np.abs(ang_diff(theta,theta_start) - np.pi) < tol:
            self.set_lap(False)
            self.set_state(State.Arrival.value)
            memory.will = MovementWill.Stay.value
            memory.time_curve = memory.time
            self.memory.prune_memory()
            memory.curve = self.memory.desired_curve
            dir = ((-1)**(memory.curve))
            self.memory.data[0] = memory
            self.vector_field.redefine(r,0,0,dir=dir)
