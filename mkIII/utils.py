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

def plot_robot_goalpose(robot_goal_pose):
    # Extract robot goal pose
    robot_gx, robot_gy, robot_gtheta = robot_goal_pose

    # Plot robot goal
    plt.plot(robot_gx, robot_gy, 'xy')

def plot_obstacles(obstacles):
    plt.plot(obstacles[:,0], obstacles[:,1], '.b')
    
def plot_trajectory(trajectory, color='r'):
    """
    Given a trajectory set (in this case a trajectory is vector of robot states)
    Plot the trajectories 
    """
    if trajectory.ndim >= 2:
        plt.plot(trajectory[:,0], trajectory[:,1], color)


def plot_trajectory_set(trajectory_set, plot_n_traj=1):
    if (trajectory_set.size != 0):
        for i in range(0, trajectory_set.shape[0], plot_n_traj):
            plot_trajectory(trajectory_set[i,:,:])