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


        ### State space: (x,y,theta, v, omega).T
        self.x_state = x_init


    def update_state(self, u_t, dt):
        """
        Equation of synchro-drive robot from DWA paper
        Link: https://www.researchgate.net/publication/3344494_The_Dynamic_Window_Approach_to_Collision_Avoidance
        """ 
        v, omega = u_t
         
        # if not np.isclose(omega, 0):
        #     # Omega != 0
        #     x_state[0] += (v / omega) * np.sin(x_state[2]) - (v / omega) * np.sin(x_state[2] + omega*dt)
        #     x_state[1] += -(v / omega) * np.cos(x_state[2]) - (v / omega) * np.cos(x_state[2] + omega*dt)
        #     x_state[2] += omega*dt
        # else: 
        #     # Omega == 0
        #     x_state[0] += v * dt * np.cos(x_state[2])
        #     x_state[1] += v * dt * np.sin(x_state[2])
        #     x_state[2] += omega * dt

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
        Motion model of robot

        u_t: Robot input, u_t = (v_t, omega_t) = (commanded linear vel, commanded angular vel)
        """
        # TODO: probably keep it as state
        x_t, y_t, theta_t, v_t, omega_t = x_state

        x_state[0] += u_t[0] * dt * np.cos(theta_t)
        x_state[1] += u_t[0] * dt * np.sin(theta_t)
        x_state[2] += u_t[1] * dt
        x_state[3] = u_t[0]
        x_state[4] = u_t[1]

        return x_state


# if __name__ == "__main__":
#     ### PARAMS
#     DT = 0.05        # Delta time
#     MAP_MIN_X = -5
#     MAP_MAX_X = 5
#     MAP_MIN_Y = -5
#     MAP_MAX_Y = 5
#     NUM_OBSTACLES = 10

#     # Robot stuff
#     INITIAL_ROBOT_STATE = np.array([0.0, 0.0, np.pi/2, 0.0, 0.0])     # Robot initial state [x,y,yaw,v,omega].T


#     ### Map Creation
#     world_map = Map(min_x=MAP_MIN_X, max_x=MAP_MAX_X, min_y=MAP_MIN_Y, max_y=MAP_MAX_Y,
#                     num_obstacles=NUM_OBSTACLES)

#     ### Robot Creation
#     robot_config = {
#                     "minimum_velocity": -0.5,
#                     "maximum_velocity": 5,
#                     "minimum_omega": -np.pi,
#                     "maximum_omega": np.pi,
#                     "maximum_acceleration": 2,
#                     "maximum_angular_acceleration": np.deg2rad(40), 

#                     "v_resolution": 0.2,
#                     "omega_resolution": np.deg2rad(2),

#                     "robot_type": "circle",
#                     "robot_radius": 0.2
#                     }

#     # De robot itself!
#     fido_robot = DifferentialRobot(robot_config=robot_config, x_init=INITIAL_ROBOT_STATE)

#     ### Circular Trajectory Generation Test
#     vel_search_space = np.arange(fido_robot.min_vel, fido_robot.max_vel, fido_robot.v_resolution)
#     omega_search_space = np.arange(fido_robot.min_omega, fido_robot.max_omega, fido_robot.omega_resolution)

#     # v_min = 0
#     # v_max = 2
#     # v_res = 0.2
#     # num_possible_v = int((v_max - v_min) / v_res)

#     # omega_min = -np.pi
#     # omega_max = np.pi
#     # omega_res = np.pi / 6
#     # num_possible_omega = int((omega_max - omega_min) / omega_res)

#     # vel_search_space = np.linspace(v_min, v_max, num_possible_v)
#     # omega_search_space = np.linspace(omega_min, omega_max, num_possible_omega)

#     print("V_space: \n", vel_search_space)
#     print("omega_space: \n", np.flip(omega_search_space))


#     ## Generate n step trajectory set for testing
#     control_input = np.array([1.0, 0.2])
#     x_state = INITIAL_ROBOT_STATE
#     n_step = 20

#     trajectory_set = np.zeros((0, n_step+1, x_state.shape[0]))  # +1 on nstep to store current position


#     for v in vel_search_space:
#         for omega in omega_search_space:
#             control_input = np.array([v, omega])

            
#             trajectory = fido_robot.generate_nstep_trajectory(x_state, control_input, DT, n_timestep_horizon=n_step)
    
#             trajectory_set = np.vstack((trajectory_set, trajectory[None]))
            

#     ### Plot trajectory
#     plot_robot(fido_robot)
#     plot_trajectory_set(trajectory_set)
#     plt.axis("equal")
#     # plt.xlim(-5, 5)
#     # plt.ylim(-2, 5)
#     plt.show()



