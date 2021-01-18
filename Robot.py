import numpy as np
from copy import deepcopy

from Map import *
from utils import *

class DifferentialRobot():
    """
    Unicycle Motion Model Robot
    EQN taken from: https://www.youtube.com/watch?v=aE7RQNhwnPQ
    --> Note that a first order derivative approximation was used to get the state space

    """
    def __init__(self, robot_config, x_init):
        ### Robot specification
        self.min_vel = robot_config["minimum_velocity"]             # Min. linear velocity
        self.max_vel = robot_config["maximum_velocity"]             # Max. linear velocity
        self.min_omega = robot_config["minimum_omega"]              # Min. Angular velocity
        self.max_omega = robot_config["maximum_omega"]              # Max. angular velocity
        self.max_acc = robot_config["maximum_acceleration"]         # Max. linear acceleration
        self.max_ang_acc = robot_config["maximum_angular_acceleration"] # Max. angular acceleration

        self.v_resolution = robot_config["v_resolution"]                # Linear velocity resolution
        self.omega_resolution = robot_config["omega_resolution"]            # Angular velocity resolution

        self.robot_type = robot_config["robot_type"]                # Robot shape specification (2D)
        self.robot_radius = robot_config["robot_radius"]

        ### State space: (x,y,theta, v, omega).T
        self.x_state = x_init


    def update_state(self, u_t, dt):
        """
        Updates the state of the robot given its equation of motion
        """ 
        v, omega = u_t
        # Store inputs into state space
        self.x_state[3] = v
        self.x_state[4] = omega

        # Update states with motion model
        self.x_state[0] += v * dt * np.cos(self.x_state[2])
        self.x_state[1] += v * dt * np.sin(self.x_state[2])
        self.x_state[2] += omega * dt

        return self.x_state

    def predict_pose(self, robot_pose, u_t, dt):
        next_pose = np.zeros(3)
        v, omega = u_t

        next_pose[0] = robot_pose[0] + v * dt * np.cos(robot_pose[2])
        next_pose[1] = robot_pose[1] + v * dt * np.sin(robot_pose[2])
        next_pose[2] = robot_pose[2] + omega * dt

        return next_pose

    def motion_model(self, x_state, u_t, dt):
        """
        Motion model of robot. 

        Note this function is only used for prediction for future trajectories
        (so it doesn't update states),

        u_t: Robot input, u_t = (v_t, omega_t) = (commanded linear vel, commanded angular vel)
        """
        x_t, y_t, theta_t, v_t, omega_t = x_state

        x_state[0] += u_t[0] * dt * np.cos(theta_t)
        x_state[1] += u_t[0] * dt * np.sin(theta_t)
        x_state[2] += u_t[1] * dt
        x_state[3] = u_t[0]
        x_state[4] = u_t[1]

        return x_state


