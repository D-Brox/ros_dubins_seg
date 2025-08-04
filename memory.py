#!/usr/bin/env python
import numpy as np
from state import MovementWill, State

class MemoryItem():
    def __init__(self,number = 0,group = 0):
        self.group = group
        self.number = number
        self.curve = 0
        self.time = 0
        self.time_curve = 0
        self.pose = [0,0,0]
        self.will = MovementWill.Stay.value
        self.state = State.InCircle.value

    def update(self, update_data):
        self.group = update_data.group
        self.number = update_data.number
        self.curve = update_data.curve
        self.time
        self.time_curve = update_data.time_curve
        self.pose = update_data.pose
        self.will = update_data.will
        self.state = update_data.state

class RobotMemory():
    def __init__(self,group,number,d,c,Rb,v):
        self.data = [MemoryItem(group,number)]
        self.neighbors = []
        self.theta_curve = 0
        self.desired_curve = 0
        self.lap = False
        self.d = d
        self.c = c
        self.Rb = Rb
        self.v = v

    def update_self(self,pose,time, arrive = False):
        self.data[0].time = time
        self.data[0].pose = pose
        self.data[0].curve = max(round(np.sqrt(pose[0]**2 + pose[1]**2)/self.d), 1)
        if arrive:
            self.data[0].time_curve = time

    def update_others(self,memories,neighbors):
        self.neighbors = neighbors
        for n in neighbors:
            self.compare_and_update(memories[n].data)

    def reset(self):
        self.data = [self.data[0]]

    def prune_memory(self):
        neighbors = [other for other in self.data[1:] if other.number in self.neighbors]
        self.data = [self.data[0]] + neighbors

    def update_memory_about_itself(self, i_data):
        self.data[0].update(i_data)

    def itself(self):
        return self.data[0]

    def others(self):
        return self.data[1:]

    def neighborhood(self):
        return [other for other in self.data[1:] if other.number in self.neighbors]

    def get_memory(self):
        return self.data

    def set_neighbors(self,neighbors):
        self.neighbors = neighbors

    def check_and_update_memory_about_j(self, j_data):
        if not j_data.curve:
            # Not initialized, curve = 0
            return
        j_in_data = False
        i_data = self.data[0]
        for memory_idx in range(1,len(self.data)):
            if self.data[memory_idx].number == j_data.number:
                j_in_data = True
                if self.data[memory_idx].time < j_data.time:
                    self.data[memory_idx].update(j_data)
        if (not j_in_data) and j_data.number != i_data.number:
            new_item = MemoryItem()
            new_item.update(j_data)
            self.data.append(new_item)

    def compare_and_update(self, other_memory_data):
        for item in other_memory_data:
            self.check_and_update_memory_about_j(item)


