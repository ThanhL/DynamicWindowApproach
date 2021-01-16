import argparse
import numpy as np
import matplotlib.pyplot as plt

from copy import deepcopy

from Robot import *
from Map import *
from DWAPlanner import *
from utils import *

### Global simulation params
MAX_SIM_TIME = 1000

DT = 0.05        # Delta time
MAP_MIN_X = -5
MAP_MAX_X = 5
MAP_MIN_Y = -5
MAP_MAX_Y = 5
NUM_OBSTACLES = 50

# Robot stuff
INITIAL_ROBOT_STATE = np.array([0.0, 0.0, np.pi/2, 0.0, 0.0])     # Robot initial state [x,y,yaw,v,omega].T
GOAL_POSE = np.array([3.0, 0.0, np.pi/2])

### Simulator Crux
def run_sim(robot, world_map, robot_goal_pose, planner, dt=DT, render=True):
    print("[!] Running Dynamic Window Approach Simulation...")

    ### Plot initial environment
    if render:
        # Clear the figures
        plt.clf()

        # Plot environment assets
        plot_obstacles(world_map.obstacles)
        plot_robot(robot)
        plot_robot_goalpose(robot_goal_pose)

        # Nearest obstalces
        nearest_obstacles = planner.calculate_nearest_obstacles(robot.x_state, world_map.obstacles)
        print("Nearest obstacles: ", nearest_obstacles)

        # Figure plot settings
        plt.axis("equal")
        plt.xlim([world_map.min_x + np.sign(world_map.min_x) * 2,       # Added extra buffer for x,y axes limit 
                world_map.max_x + np.sign(world_map.max_x) * 2])
        plt.ylim([world_map.min_y + np.sign(world_map.min_y) * 2, 
                world_map.max_y + np.sign(world_map.max_y) *2])
        plt.grid(True)
        plt.waitforbuttonpress()


    ### Run DWA planner
    # Simulation start time
    time = 0.0



    while time <= MAX_SIM_TIME:     # TODO: probably remove this for a "reach goal" check
        print("[+] Elapsed sim_time: {:.3f}".format(time))

        # Update sim time
        time += DT
        
        ### DWA control
        u_t_dwa, best_traj, trajectory_set = planner.calc_dwa_control(robot.x_state, robot_goal_pose,
                                                                    world_map.obstacles)
        robot.update_state(u_t_dwa, DT)

        if render:
            # Clear the figures
            plt.clf()

            plot_obstacles(world_map.obstacles)
            plot_robot(robot)
            plot_robot_goalpose(robot_goal_pose)

            plot_trajectory_set(trajectory_set)
            plot_trajectory(best_traj, color='g')

            # Plot details
            plt.axis("equal")
            plt.xlim([world_map.min_x + np.sign(world_map.min_x) * 2,       # Added extra buffer for x,y axes limit 
                    world_map.max_x + np.sign(world_map.max_x) * 2])
            plt.ylim([world_map.min_y + np.sign(world_map.min_y) * 2, 
                    world_map.max_y + np.sign(world_map.max_y) *2])
            plt.grid(True)
            #plt.pause(0.0001)

            plt.pause(0.00001)
            # plt.waitforbuttonpress()


def main():
    ### Map Creation
    world_map = Map(min_x=MAP_MIN_X, max_x=MAP_MAX_X, min_y=MAP_MIN_Y, max_y=MAP_MAX_Y,
                    num_obstacles=NUM_OBSTACLES)

    ### Robot Creation
    robot_config = {
                    "minimum_velocity": -0.0,
                    "maximum_velocity": 2,
                    "minimum_omega": -2.84,
                    "maximum_omega": 2.84,
                    "maximum_acceleration": 1.2,
                    "maximum_angular_acceleration": np.deg2rad(40), 

                    "v_resolution": 0.05,
                    "omega_resolution": np.deg2rad(0.25),

                    "robot_type": "circle",
                    "robot_radius": 0.2
                    }

    robot_initial_state = INITIAL_ROBOT_STATE

    # De robot itself!
    fido_robot = DifferentialRobot(robot_config=robot_config, x_init=robot_initial_state)

    ### Planner Config
    planner_config = {
                    "alpha": 0.8,
                    "beta": 0.8,
                    "gamma": 0.4,

                    "delta_time": DT,
                    "n_horizon": 60
                    # "n_horizon": 60
                    }

    dwa_planner = DWAPlanner(planner_config=planner_config,
                            robot=fido_robot)

    ### Running the simulation
    run_sim(robot=fido_robot, world_map=world_map, robot_goal_pose=GOAL_POSE,
        planner=dwa_planner)
    

if __name__ == "__main__":
    main()
