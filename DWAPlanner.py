import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

class DWAPlanner():
    def __init__(self, planner_config, robot_config, robot):
        ### Robot specification
        # TODO: note these config variables should be inside the robot planner
        # TODO: remove this? and replace with just the robot object instead?
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

        ### Planner Spefications 
        self.dt = planner_config["delta_time"]      # Delta time

        self.alpha = planner_config["alpha"]        # Goal Gain heuristic
        self.beta = planner_config["beta"]          # Distance to Obstacle Gain heuristic
        self.gamma = planner_config["gamma"]        # Forward Velocity Gain Heuristic

        # Number of timesteps for prediction into future (for trajectory generation)
        self.n_horizon = planner_config["n_horizon"]    

        ### Robot itself
        self.robot = robot

    ### Dynamic window calculations
    # Space of possible velocities
    def calculate_Vs(self):
        return [self.min_vel, self.max_vel, self.min_omega, self.max_omega]

    # Dynamic windows
    def calculate_Vd(self, robot_state):
        robot_x, robot_y, robot_theta, v_a, omega_a = robot_state
        return [v_a - self.max_acc * self.dt, v_a + self.max_acc * self.dt,               # Linear velocity
            omega_a - self.max_ang_acc * self.dt, omega_a + self.max_ang_acc * self.dt]   # Angular velocity

    # Admissiable velocites
    def calculate_Va(self, robot_state):
        # TODO: fix this calculation for Admissible Velocities
        return [self.min_vel, np.sqrt(2) * self.max_acc,                        # Linear velocity
            self.min_omega, np.sqrt(2) * self.max_ang_acc]                      # Angular velocity

    # Calculate resulting search space windows
    def calc_dyn_win(self, robot_state):
        ### Calculate Velocity spaces
        Vs = self.calculate_Vs()
        Vd = self.calculate_Vd(robot_state)
        Va = self.calculate_Va(robot_state)

        # Resulting Search space
        Vr = [max(Vs[0], Vd[0]),        # Resulting Minimum Linear velocity Vr_v_min
            min(Vs[1], Vd[1]),          # Resulting Maximum Linear velocity Vr_v_max
            max(Vs[2], Vd[2]),          # Resulting Minimum Angular velocity Vr_omega_min
            min(Vs[3], Vd[3])]          # Resulting Maximum Angular velocity Vr_omega_max


        print("Vs: ", Vs)
        print("Vd: ", Vd)
        print("Vr: ", Vr)

        Vr_v_min = max(Vs[0], Vd[0])        # Resulting Minimum Linear velocity Vr_v_min
        Vr_v_max = min(Vs[1], Vd[1])        # Resulting Maximum Linear velocity Vr_v_max
        Vr_omega_min = max(Vs[2], Vd[2])    # Resulting Minimum Angular velocity Vr_omega_min
        Vr_omega_max = min(Vs[3], Vd[3])    # Resulting Maximum Angular velocity Vr_omega_max    

        # Generate search space for velocities
        Vr_v = np.arange(Vr_v_min, Vr_v_max, self.v_resolution)
        Vr_omega = np.arange(Vr_omega_min, Vr_omega_max, self.omega_resolution)

        return Vr_v, Vr_omega

    ### Dynamic Window Approach crux (control calculations)
    # Generation of circular trajectories given control input
    def generate_trajectory(self, robot_state, u):
        x_state = deepcopy(robot_state)
        trajectory = x_state

        for i in range(self.n_horizon):
            x_state = self.robot.motion_model(x_state, u, self.dt)
            trajectory = np.vstack((trajectory, x_state))

        return trajectory


    def calc_dwa_control(self, robot_pose, robot_goal):
        """
        DWA control inputs calculation
        """
        # Best Metrics Initializer
        minimum_cost = np.inf       # Initialize minimum cost to extremely large initially
        best_control_input = np.zeros(2)    # Control input 

        # Compute the resulting search space
        Vr_v, Vr_omega = self.calc_dyn_win(robot_pose)

        # Trajectory set (store all possible circular trajectories for visualization)
        num_possible_trajectories = Vr_v.shape[0] * Vr_omega.shape[0]
        trajectory_set = np.zeros((0, self.n_horizon+1, robot_pose.shape[0]))

        # Vr_v = np.arange(0, 0.4, 0.2)
        # Vr_omega = np.arange(np.deg2rad(-30), np.deg2rad(30), np.deg2rad(2))

        # Evaluate 
        x_init = deepcopy(robot_pose)
        for v in Vr_v:
            for omega in Vr_omega:
                ### Generate predicted trajectories with (v, omega) pair from search space
                control_input = np.array([v, omega])
                trajectory = self.generate_trajectory(x_init, control_input)

                # Append predicted trajectory to set
                # print("traj set shape: ", trajectory_set.shape)
                # print("robot_shape: ", robot_pose.shape)
                # print("traj shape: ", trajectory.shape)
                trajectory_set = np.vstack((trajectory_set, trajectory[None]))

                # print("(v,r): ({:.3f}, {:.3f})".format(v, omega))
                # print("robot pose: ", x_init)
                # print("traj: ", trajectory)
                # print()

        print("Vr_v: ", Vr_v)
        print("Vr_omega: ", Vr_omega)
        print("num_possible_trajectories: ", num_possible_trajectories)
        print("robot_pose_shape: ", robot_pose.shape)
        print("trajectory_set_shape: ", trajectory_set.shape)


        return None, trajectory_set

    ### Heuristics (from DWA paper)
    # 'angle' heuristic
    def calc_obs_dist_heuristic(self):
        return

    # 'dist' heuristic
    def calc_goal_heuristic(self):
        return

    # 'vel' heuristic
    def calc_vel_heuristic(self):
        return

    # Objective function
    def calc_obj_func(self):
        return


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
                    "robot_radius": 1.,

                    }

    planner_config = {
                    "alpha": 0.15,
                    "beta": 1.0,
                    "gamma": 1.0,

                    "delta_time": 0.1,
                    "n_horizon": 5
                    }


    dwa_planner = DWAPlanner(planner_config)