import rps.robotarium as robotarium
from rps.utilities.misc import plt
from rps.utilities.controllers import create_clf_unicycle_pose_controller
from rps.utilities.graph import delta_disk_neighbors
# from rps.utilities.barrier_certificates import create_unicycle_barrier_certificate

from memory import RobotMemory
from state import State
from controller import SegregationController

import numpy as np
from numpy import pi
# from time import sleep

N = 5 # Number of Robots

d = 0.25 # space between circles
c = 1 # communication distance
Rb = 0.11/2 # robot radius
v = 1  # velocity

groups = [2,0,1,2,0]
group_list = set(groups)
n_groups = len(group_list)
initial_conditions = np.array(
[   #   C      D        E       F       G
    [   0.25, -0.25,   -0.25,  -0.25,   0.75],
    [   0.00,  0.00,   -0.43,   0.43,   0.00],
    [   pi/2, -pi/2,  5*pi/6,   pi/6,   pi/2],
]) # Initial Positions

def metric(m):
    S = [0 for i in range(N)]
    S_min = {g: np.inf for g in group_list}
    S_max = {g: 0 for g in group_list}
    x = [ m[i].data[0].pose for i in range(N)]
    for i in range(N):
        g = groups[i]
        S[i] = round(np.sqrt(x[i][0]**2 + x[i][1]**2)/d)
        if not S:
            S = 1
        S_min[g] = min(S_min[g],S[i])
        S_max[g] = max(S_max[g],S[i])
    
    infringing = 0
    for i in range(N):
        for k in group_list:
            if groups[i] != k and S_min[k]<=S[i]<=S_max[k]:
                infringing += 1
    
    return (1-infringing/(N*(n_groups-1)))


r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True) # Instantiate Robotarium object
img = plt.imread('./map.png')
x_img = np.linspace(-1.0, 1.0, img.shape[1])
y_img = np.linspace(-1.0, 1.0, img.shape[0])
gt_img_handle = r.axes.imshow(img, extent=(-1,1,-1,1))
x = r.get_poses() # Get the poses of Robots

memory_robots = [RobotMemory(i,groups[i],d,c,Rb,v) for i in range(N)]
controllers = [SegregationController() for i in range(N)]
for i in range(N):
    memory_robots[i].update_self(x.T[i],0)
    controllers[i].memory = memory_robots[i]
    controllers[i].init_vector_field()

unicycle_pose_controller = create_clf_unicycle_pose_controller() # Create unicycle pose controller
# uni_barrier_cert = create_unicycle_barrier_certificate(safety_radius=0.12) # Create barrier certificates to avoid collision
r.step() # Iterate the simulation

time = 0
segregation = 0

end_time = 5*60 
while time < end_time:
    x = r.get_poses()
    x_goals = x.copy()
    
    for i in range(N):
        memory_robots[i].update_self(x.T[i],time)
    for i in range(N):
        # Apply communication radius
        memory_robots[i].update_others(memory_robots,delta_disk_neighbors(x,i,c))

    # Calculate wills
    dxu = np.zeros((2,N))
    for i in range(N):
        controllers[i].memory = memory_robots[i]
        if time > 1:
            if controllers[i].memory.data[0].state == State.InCircle.value:
                controllers[i].calculate_lap() # l7-l8
                inward,outward = controllers[i].calculate_will() # l9-l22 and A2 at the end
                if inward or outward:
                    controllers[i].prevent_collision(inward,outward)
            # A1: l25-l
            elif controllers[i].memory.data[0].state == State.Transition.value:
                arrive = controllers[i].check_arrival()
            else:
                # Don't imediately try to move again
                if memory_robots[i].data[0].time < memory_robots[i].data[0].time + 1/2:
                    memory_robots[i].data[0].state = State.InCircle.value

            memory_robots[i].data[0] = controllers[i].memory.data[0] # may have updated in algorithms
            controllers[i].check_memory_lap() 

        x_goals[:,i] = controllers[i].compute_next(1/30)
        #dxu[:,i] = controllers[i].calculate_lower_control()

    # Already safe due to the collision prevention algorithm
    dxu = unicycle_pose_controller(x, x_goals)
    r.set_velocities(np.arange(N), dxu)
    r.step()

    time += 1/30

    

r.call_at_scripts_end()




