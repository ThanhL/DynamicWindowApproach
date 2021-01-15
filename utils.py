import numpy as np
import matplotlib.pyplot as plt 

def plot_robot(robot):
    # Extract robot_pose
    robot_x, robot_y, robot_theta, _, _ = robot.x_state

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
    robot_goal_x, robot_goal_y, robot_goal_theta, _, _ = robot_goalstate

    # Plot robot goal
    plt.plot(robot_goal_x, robot_goal_y, 'xy')

def plot_obstacles(obstacles):
    plt.plot(obstacles[:,0], obstacles[:,1], 'sb')


def plot_trajectories(robot_traj_start, robot_traj_goal):
    robot_start_x, robot_start_y = robot_traj_start
    robot_end_x, robot_end_y = robot_traj_goal

    delta_x = robot_end_x - robot_start_x
    delta_y = robot_end_y - robot_start_y

    plt.arrow(robot_start_x, robot_start_y, delta_x, delta_y)
    
def plot_trajectory(trajectory):
    """
    Given a trajectory set (in this case a trajectory is vector of robot states)
    Plot the trajectories 
    """
    plt.plot(trajectory[:,0], trajectory[:,1], 'r')

    # for i in range(trajectory.shape[0] - 1):
    #     delta_x, delta_y, _ = trajectory[i] - trajectory[i+1]

    #     plt.arrow(trajectory[i,0], trajectory[i,1], delta_x, delta_y, color='y')

def plot_trajectory_set(trajectory_set):
    # print("trajectory_set: \n", trajectory_set)

    for i in range(trajectory_set.shape[0]):
        # print("trajectory_i: ", trajectory_set[i,:,:])
        plot_trajectory(trajectory_set[i,:,:])