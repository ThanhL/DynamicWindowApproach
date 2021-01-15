import argparse
import numpy as np
import matplotlib.pyplot as plt

from Robot import *
from Map import *
from utils import *

### Global simulation params
MAX_SIM_TIME = 20

DT = 0.1        # Delta time
MAP_MIN_X = -5
MAP_MAX_X = 5
MAP_MIN_Y = -5
MAP_MAX_Y = 5

# Robot stuff
INITIAL_ROBOT_STATE = np.array([0.0, 0.0, np.pi/2])     # Robot initial state [x,y,yaw].T
GOAL_POSITION = np.array([3.0, 3.0])
GOAL_STATE = np.array([3.0, 3.0, np.pi/2])

### Simulator Crux
def run_sim(robot, world_map, goal_state, dt=DT, render=True):
    print("[!] Running Dynamic Window Approach Simulation...")

    # Simulation start time
    time = 0.0


    while time <= MAX_SIM_TIME:     # TODO: probably remove this for a "reach goal" check
        print("[+] Elapsed sim_time: {:.3f}".format(time))

        # Update sim time
        time += DT
        
        # TODO: Remove. Dummy input for now
        u_t = np.array([0.1, 0.1])
        robot.x_state = robot.motion_model(robot.x_state, u_t, DT)

        if render:
            # Clear the figures
            plt.clf()

            plot_obstacles(world_map.obstacles)
            plot_robot(robot)
            plot_robot_goalstate(goal_state)

            # Plot details
            plt.axis("equal")
            plt.xlim([world_map.min_x + np.sign(world_map.min_x) * 2,       # Added extra buffer for x,y axes limit 
                    world_map.max_x + np.sign(world_map.max_x) * 2])
            plt.ylim([world_map.min_y + np.sign(world_map.min_y) * 2, 
                    world_map.max_y + np.sign(world_map.max_y) *2])
            plt.grid(True)
            plt.pause(0.0001)


def main():
    print("[!] Running Dynamic Window Approach Simulation...")

    ### Map Creation
    world_map = Map(min_x=MAP_MIN_X, max_x=MAP_MAX_X, min_y=MAP_MIN_Y, max_y=MAP_MAX_Y)

    ### Robot Creation
    robot_config = {
                    "minimum_velocity": 0.1,
                    "maximum_velocity": 0.22,
                    "minimum_omega": 0.1,
                    "maximum_omega": 2.84,
                    "maximum_acceleration": 0.2,
                    "maximum_angular_acceleration": np.deg2rad(40), 

                    "v_resolution": 0.02,
                    "omega_resolution": np.deg2rad(0.1),

                    "robot_type": "circle",
                    "robot_radius": 0.2
                    }

    robot_initial_state = np.array([0.0, 0.0, np.deg2rad(90.0)])

    # De robot itself!
    fido_robot = DifferentialRobot(robot_config=robot_config, x_init=robot_initial_state)

    ### Running the simulation
    run_sim(robot=fido_robot, world_map=world_map, goal_state=GOAL_STATE)
    

if __name__ == "__main__":
    main()