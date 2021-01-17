import numpy as np
import matplotlib.pyplot as plt 

### Environment related plots
def plot_robot(robot):
    """
    Plots the robot at (x,y,theta)
    """
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

def plot_robot_goalpose(robot_goal_pose):
    """
    Plots the robot goal position
    """
    # Extract robot goal pose
    robot_gx, robot_gy, robot_gtheta = robot_goal_pose

    # Plot robot goal
    plt.plot(robot_gx, robot_gy, 'Dc')

def plot_obstacles(obstacles):
    """
    Plots the obstacles of the world map
    """
    plt.plot(obstacles[:,0], obstacles[:,1], '.b')

def plot_sim_environment(robot, robot_goal_pose, world_map):
    """ 
    Plots the robot, robot's goal pose, and the world map's obstacles
    """
    # Plot simulator assets
    plot_obstacles(world_map.obstacles)
    plot_robot(robot)
    plot_robot_goalpose(robot_goal_pose)

    # Plot settings
    plt.axis("equal")
    plt.grid(True)

### DWA Trajectory Plots   
def plot_trajectory(trajectory, color='r'):
    """
    Plots the trajectory 
    """
    if trajectory.ndim >= 2:
        plt.plot(trajectory[:,0], trajectory[:,1], color)

def plot_trajectory_set(trajectory_set, plot_n_traj=1):
    """
    Given a trajectory set, plot the trajectories
    """
    if (trajectory_set.size != 0):
        for i in range(0, trajectory_set.shape[0], plot_n_traj):
            plot_trajectory(trajectory_set[i,:,:])