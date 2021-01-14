import numpy as np


class DifferentialRobot():
    """
    Unicycle Motion Model Robot
    EQN taken from: https://www.youtube.com/watch?v=aE7RQNhwnPQ
    --> Note that a first order derivative approximation was used to get the state space

    """
    def __init__(self, robot_config, x_init):
        ### Robot specification
        self.min_speed = robot_config["minimum_speed"]
        self.max_speed = robot_config["maximum_speed"]
        self.max_rot_speed = robot_config["maximum_rotational_speed"]

        self.v_resolution = robot_config["speed_resolution"]
        self.omega_resolution = robot_config["omega_resolution"]

        ### State space: (x,y,theta)
        self.x_state = x_init

    def motion_model(self, u_t, dt):
        """
        Motion model of robot

        u_t: Robot input, u_t = (v_t, omega_t) = (commanded linear vel, commanded angular vel)
        """
        x_t, y_t, theta_t = self.x_state

        self.x_state[0] += u_t[0] * dt * np.cos(theta_t)
        self.x_state[1] += u_t[0] * dt * np.sin(theta_t)
        self.x_state[2] += u_t[1] * dt



if __name__ == "__main__":
    robot_config = {
                    "maximum_speed": 0.22,
                    "maximum_rotational_speed": 2.84
                    }
    x_init = np.array([0.0, 0.0, 0.0])

    turtlebot3 = DifferentialRobot(
