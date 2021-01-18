import yaml
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


### Simulator Crux
def run_sim(robot, world_map, robot_goal_pose, planner, goal_dist_thresh=0.2, 
        dt=0.05, max_sim_time=1000, render=True):
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

    while time <= max_sim_time and not reached_goal:
        print("[+] Elapsed sim_time: {:.3f}".format(time))

        ### Update sim time
        time += dt
        
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
            robot.update_state(u_t_dwa, dt)

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
    parser.add_argument("--robot_config", type=str, default="./config/robot_config.yaml")
    parser.add_argument("--planner_config", type=str, default="./config/planner_config.yaml")
    parser.add_argument("--sim_config", type=str, default="./config/sim_config.yaml")

    # Extract user input
    args = parser.parse_args()
    start_position = tuple(args.start_position)
    end_position = tuple(args.end_position)

    robot_config_file = args.robot_config
    planner_config_file = args.planner_config
    sim_config_file = args.sim_config

    ### Extract & load configuration files
    with open(robot_config_file) as robot_config_file:
        robot_config = yaml.load(robot_config_file, Loader=yaml.FullLoader)

    with open(planner_config_file) as planner_config_file:
        planner_config = yaml.load(planner_config_file, Loader=yaml.FullLoader)

    with open(sim_config_file) as sim_config_file:
        sim_config = yaml.load(sim_config_file, Loader=yaml.FullLoader)


    ## Set initial robot pose and goal pose
    robot_pose = np.array([start_position[0], start_position[1], np.deg2rad(start_position[2])])
    goal_pose = np.array([end_position[0], end_position[1], np.deg2rad(end_position[2])])

    ### Map Creation
    world_map = Map(start_pose=robot_pose, end_pose=goal_pose,
                    min_x=sim_config["map_min_x"], 
                    max_x=sim_config["map_max_x"], 
                    min_y=sim_config["map_min_y"], 
                    max_y=sim_config["map_max_y"],
                    num_obstacles=sim_config["num_obstacles"])

    ### Robot Creation
    # Robot's initial state: (x,y,theta,v_a,omega_a).T
    robot_initial_state = np.array([robot_pose[0], robot_pose[1], robot_pose[2], 0., 0.])

    # De robot itself!
    fido_robot = DifferentialRobot(robot_config=robot_config, x_init=robot_initial_state)

    ### DWA Planner
    dwa_planner = DWAPlanner(planner_config=planner_config,
                            robot=fido_robot)

    ### Running the simulation
    run_sim(robot=fido_robot, 
        world_map=world_map, 
        robot_goal_pose=goal_pose,
        planner=dwa_planner, 
        dt=sim_config["delta_time"],
        max_sim_time=sim_config["max_sim_time"])
    

if __name__ == "__main__":
    main()

