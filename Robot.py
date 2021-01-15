import numpy as np


class DifferentialRobot():
    """
    Unicycle Motion Model Robot
    EQN taken from: https://www.youtube.com/watch?v=aE7RQNhwnPQ
    --> Note that a first order derivative approximation was used to get the state space

    """
    def __init__(self, robot_config, x_init):
        ### Robot specification
        # TODO: note these config variables should be inside the robot planner
        self.min_vel = robot_config["minimum_velocity"]             # Min. linear velocity
        self.max_vel = robot_config["maximum_velocity"]             # Max. linear velocity
        self.min_omega = robot_config["minimum_omega"]              # Min. Angular velocity
        self.max_omega = robot_config["maximum_omega"]              # Max. angular velocity
        self.max_acc = robot_config["maximum_acceleration"]         # Max. linear acceleration
        self.max_ang_acc = robot_config["maximum_angular_acceleration"] # Max. angular acceleration

        self.v_resolution = robot_config["v_resolution"]                # Linear velocity resolution
        self.omega_resolution = robot_config["omega_resolution"]            # Angular velocity resolution

        self.robot_type = robot_config["robot_type"]                        # Robot shape specification (2D)

        # TODO: put this into a Class type? --> robot specs?
        self.robot_radius = robot_config["robot_radius"]


        ### State space: (x,y,theta).T
        self.x_state = x_init



    def motion_model(self, x_state, u_t, dt):
        """
        Motion model of robot

        u_t: Robot input, u_t = (v_t, omega_t) = (commanded linear vel, commanded angular vel)
        """
        # TODO: probably keep it as state
        x_t, y_t, theta_t = x_state

        x_state[0] += u_t[0] * dt * np.cos(theta_t)
        x_state[1] += u_t[0] * dt * np.sin(theta_t)
        x_state[2] += u_t[1] * dt

        return x_state




if __name__ == "__main__":
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
                    "robot_radius": 1.
                    }

    x_init = np.array([0.0, 0.0, 0.0])


