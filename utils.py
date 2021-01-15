import numpy as np
import matplotlib.pyplot as plt 

def plot_robot(robot):
    # Extract robot_pose
    robot_x, robot_y, robot_theta = robot.x_state

    if robot.robot_type == "circle":
        # plot robot circle position
        robot_circle = plt.Circle((robot_x, robot_y), robot.robot_radius, color="k")
        plt.gcf().gca().add_artist(robot_circle)

        # plot robot front facing direction (represented by a bar)
        robot_front_x, robot_front_y = (np.array([robot_x, robot_y]) + 
                                    np.array([np.cos(robot_theta), np.sin(robot_theta)]) * robot.robot_radius)
        plt.plot([robot_x, robot_front_x], [robot_y, robot_front_y], "-w")

def plot_robot_goalstate(robot_goalstate, robot_type="circle"):
    # Extract robot goal
    robot_goal_x, robot_goal_y, robot_goal_theta = robot_goalstate

    # Plot robot goal
    plt.plot(robot_goal_x, robot_goal_y, 'xy')

def plot_obstacles(obstacles):
    plt.plot(obstacles[:,0], obstacles[:,1], 'sb')


def plot_trajectories(traj):
    return
