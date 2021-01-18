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
NUM_OBSTACLES = 60

### Simulator Crux
def run_sim(robot, world_map, robot_goal_pose, planner, goal_dist_thresh=0.2, dt=DT, render=True):
    """ 
    Simulates Dynamic Window Approach algorithm
    """
    print("[!] Running Dynamic Window Approach Simulation...")

    ### Plot initial environment
    if render:
        plt.clf()
        plot_sim_environment(robot, robot_goal_pose, world_map)
        plt.legend(["obstacles", "robot", "goal"], loc='lower left')

        print("--- Press the any key on the figure to start the simulation ---")
        plt.waitforbuttonpress()


    ### Run DWA planner
    reached_goal = False
    time = 0.0          # Simulation start time

    while time <= MAX_SIM_TIME and not reached_goal:
        print("[+] Elapsed sim_time: {:.3f}".format(time))

        ### Update sim time
        time += DT
        
        ### Goal Check
        dist_to_goal = np.linalg.norm(robot.x_state[0:2] - robot_goal_pose[0:2])
        if  dist_to_goal < goal_dist_thresh:
            ### We've reached the goal, stop sending controls to the robot!
            print("[!] Reached Goal")
            reached_goal = True
        else:
            ### DWA control 
            u_t_dwa, best_traj, trajectory_set = planner.calc_dwa_control(robot.x_state, robot_goal_pose,
                                                                        world_map.obstacles)
            robot.update_state(u_t_dwa, DT)

        if render:
            plt.clf()
            plot_trajectory_set(trajectory_set, plot_n_traj=20)
            plot_trajectory(best_traj, color='limegreen')
            plot_sim_environment(robot, robot_goal_pose, world_map)
            
            plt.legend(["obstacles", "robot", "goal"], loc='lower left')
            plt.pause(0.00001)

    ### Closure
    if not reached_goal:
        print("[!] Robot did not reach the goal :(")

    input("--- Press any key to end simulation! ---")
    exit()

### Main
def main():
    ### Option parser
    parser = argparse.ArgumentParser()

    # DWA simulation parameters
    parser.add_argument("--start_position", nargs=3, type=float, required=True, 
                    help='start position (x,y,theta)')
    parser.add_argument("--end_position", nargs=3, type=float, required=True, 
                    help='end position (x,y,theta)')

    # Extract user input
    args = parser.parse_args()
    start_position = tuple(args.start_position)
    end_position = tuple(args.end_position)


    ## Set initial robot pose and goal pose
    robot_pose = np.array([start_position[0], start_position[1], np.deg2rad(start_position[2])])
    goal_pose = np.array([end_position[0], end_position[1], np.deg2rad(end_position[2])])

    ### Map Creation
    world_map = Map(start_pose=robot_pose, end_pose=goal_pose,
                    min_x=MAP_MIN_X, max_x=MAP_MAX_X, min_y=MAP_MIN_Y, max_y=MAP_MAX_Y,
                    num_obstacles=NUM_OBSTACLES)

    ### Robot Creation
    robot_config = {
                    "minimum_velocity": 0,
                    "maximum_velocity": 2,
                    "minimum_omega": -np.pi/2,
                    "maximum_omega": np.pi/2,
                    "maximum_acceleration": 2.5,
                    "maximum_angular_acceleration": np.deg2rad(60), 

                    "v_resolution": 0.1,
                    "omega_resolution": np.deg2rad(0.25),

                    "robot_type": "circle",
                    "robot_radius": 0.2
                    }

    # Robot's initial state: (x,y,theta,v_a,omega_a).T
    robot_initial_state = np.array([robot_pose[0], robot_pose[1], robot_pose[2], 0., 0.])

    # De robot itself!
    fido_robot = DifferentialRobot(robot_config=robot_config, x_init=robot_initial_state)

    ### DWA Planner
    planner_config = {
                    "alpha": 0.8,
                    "beta": 0.8,
                    "gamma": 0.6,

                    "delta_time": DT,
                    "n_horizon": 20,

                    "obstacle_distance_tolerance": 5,
                    "stuck_space_tolerance": 0.001,

                    "escape_angular_velocity": np.pi / 4
                    }

    dwa_planner = DWAPlanner(planner_config=planner_config,
                            robot=fido_robot)

    ### Running the simulation
    run_sim(robot=fido_robot, world_map=world_map, robot_goal_pose=goal_pose,
        planner=dwa_planner)
    

if __name__ == "__main__":
    main()

