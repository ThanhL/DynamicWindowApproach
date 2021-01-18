import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

class DWAPlanner():
    def __init__(self, planner_config, robot):
        ### Robot 
        self.robot = robot

        ### Planner Spefications 
        self.dt = planner_config["delta_time"]      # Delta time

        self.alpha = planner_config["alpha"]        # Goal Gain heuristic
        self.beta = planner_config["beta"]          # Distance to Obstacle Gain heuristic
        self.gamma = planner_config["gamma"]        # Forward Velocity Gain Heuristic

        # Number of timesteps for prediction into future (for trajectory generation)
        self.n_horizon = planner_config["n_horizon"]    

        # Distance tolerance to obstacle (for trajectories)
        self.obstacle_dist_tol = planner_config["obstacle_distance_tolerance"]

        # 'stuck space' tolerance: the magnitude tolerance for when robot gets stuck @ 0 vel 
        # (search space & the robot's actual velocity)
        self.stuck_space_tol = planner_config["stuck_space_tolerance"]

        # Escape velocity from 0 velocity 'stuck space'
        self.escape_ang_vel = planner_config["escape_angular_velocity"]

    ### Dynamic window calculations
    # Vs: Space of possible velocities
    def calculate_Vs(self):
        """
        Calculates the velocity search space ranges (Vs from DWA paper)
        """
        return [self.robot.min_vel, self.robot.max_vel, self.robot.min_omega, self.robot.max_omega]

    # Vd: Dynamic window for velocities
    def calculate_Vd(self, robot_state):
        """
        Calculates the dynamic window search space ranges (Vd from DWA paper)
        Implementation of EQN 15.
        """
        robot_x, robot_y, robot_theta, v_a, omega_a = robot_state
        return [v_a - self.robot.max_acc * self.dt, v_a + self.robot.max_acc * self.dt,               # Linear velocity
            omega_a - self.robot.max_ang_acc * self.dt, omega_a + self.robot.max_ang_acc * self.dt]   # Angular velocity

    # Va: Admissable velocities
    def is_admissable(self, trajectory, control_input, nearest_obstacles):
        """
        Calculates the admissable velocities search space ranges (Va from DWA paper)

        Given the obstacles and accelerations for breakage, the admissable velocities are calculated
        using EQN 14. (v, omega) pairs are checked to see if they satisfy EQN 14.
        """
        ### Extract control input
        v, omega = control_input

        ### Calculation if admissable
        obs_heuristic_dist = self.calc_obs_dist_heuristic(trajectory, nearest_obstacles)

        v_admissable_criteria = np.sqrt(2 * obs_heuristic_dist * self.robot.max_acc)
        omega_admissable_criteria = np.sqrt(2 * obs_heuristic_dist * self.robot.max_ang_acc)

        if (v <= v_admissable_criteria) and (omega <= omega_admissable_criteria):
            return True
        else:
            return False

    # Vr = INTERSECTION(Va,Vr,Vs): Calculate resulting search space windows
    def calculate_Vr(self, robot_state):
        """
        Calculates the resulting velocity search space 

        Implements EQN 16, INTERSECT(Vs, Vd). Note the admissable velocity Va is checked after
        iterating through the (v,omega) pairs of the search space (Vr_v, Vr_omega).
        """
        ### Calculate Velocity spaces
        Vs = self.calculate_Vs()
        Vd = self.calculate_Vd(robot_state)

        ### Resulting search space range
        Vr_v_min = max(Vs[0], Vd[0])        # Resulting Minimum Linear velocity Vr_v_min
        Vr_v_max = min(Vs[1], Vd[1])        # Resulting Maximum Linear velocity Vr_v_max
        Vr_omega_min = max(Vs[2], Vd[2])    # Resulting Minimum Angular velocity Vr_omega_min
        Vr_omega_max = min(Vs[3], Vd[3])    # Resulting Maximum Angular velocity Vr_omega_max    

        # Generate search space for velocities
        Vr_v = np.arange(Vr_v_min, Vr_v_max, self.robot.v_resolution)
        Vr_omega = np.arange(Vr_omega_min, Vr_omega_max, self.robot.omega_resolution)

        return Vr_v, Vr_omega

    ### Dynamic Window Approach crux (control calculations)
    # Generation of circular trajectories given control input
    def generate_trajectory(self, x, u):
        """
        Generates circular trajectories given control input pair (v,omega) for n future timesteps
        into the future. 
        """
        x_state = deepcopy(x)
        trajectory = x_state

        for i in range(self.n_horizon):
            x_state = self.robot.motion_model(x_state, u, self.dt)
            trajectory = np.vstack((trajectory, x_state))

        return trajectory

    def calc_dwa_control(self, robot_state, robot_goal, obstacles):
        """
        Calculates the dynamic window approach control inputs required for path planning.

        Returns the best control inputsm best trajectory and the resulting trajectory found
        from evaluating the velocity search space.
        """ 
        # Best Metrics Initializer
        minimum_cost = np.inf               # Initialize minimum cost to extremely large initially
        best_control_input = np.zeros(2) 
        best_trajectory = deepcopy(robot_state)

        # Compute the resulting velocity search space
        Vr_v, Vr_omega = self.calculate_Vr(robot_state)

        # Trajectory set (store all possible circular trajectories for visualization)
        num_possible_trajectories = Vr_v.shape[0] * Vr_omega.shape[0]
        trajectory_set = np.zeros((0, self.n_horizon+1, robot_state.shape[0]))

        ### Evaluate (v,omega) pairs and searches for the best control input + trajectory
        x_init = deepcopy(robot_state)
        for v in Vr_v:
            for omega in Vr_omega:
                ### Generate predicted trajectories with (v, omega) pair from search space
                control_input = np.array([v, omega])
                trajectory = self.generate_trajectory(x_init, control_input)

                ### Evaluate the paths for nearest obstacles
                nearest_obstacles = self.calculate_nearest_obstacles(robot_state, obstacles)

                ### Check if velocity is admissable
                if self.is_admissable(trajectory, control_input, nearest_obstacles):
                    trajectory_set = np.vstack((trajectory_set, trajectory[None]))

                    ### Cost calculation
                    angle_cost = self.alpha * self.calc_goal_heuristic(trajectory, robot_goal)
                    dist_cost = self.beta * self.calc_obs_dist_heuristic(trajectory, nearest_obstacles)
                    vel_cost = self.gamma * self.calc_vel_heuristic(trajectory, self.robot.max_vel)
                    
                    # Total cost
                    total_cost = angle_cost + dist_cost + vel_cost

                    ### Update best costs & store the best control inputs + trajectory
                    if minimum_cost >= total_cost:
                        # print("[!] Best Found (v,w): ({:.3f}, {:.3f})".format(v, omega))
                        minimum_cost = total_cost
                        best_control_input[:] = control_input
                        best_trajectory = trajectory

        # print("[!] Best Found (v,w): ({:.3f}, {:.3f}) \tCost: {:.3f}".format(v, omega, minimum_cost))

        ### Prevention of getting stuck in (v,omega) = 0 search space
        if (abs(best_control_input[0]) < self.stuck_space_tol and abs(robot_state[3]) < self.stuck_space_tol):
            print("[!] Robot stuck in 0 velocity, sending max spin to get out of region.")
            best_control_input[1] = self.escape_ang_vel

        # print("robot state: ", robot_state)
        # print("best_control_input: ", best_control_input)
        # print("minimum_cost: ", minimum_cost)
        # print("Vr_v: ", Vr_v)
        # print("Vr_omega: ", Vr_omega)
        # print("num_possible_trajectories: ", num_possible_trajectories)
        # print("robot_state_shape: ", robot_state.shape)
        # print("trajectory_set_shape: ", trajectory_set.shape)


        return best_control_input, best_trajectory, trajectory_set

    ### Heuristics (from DWA paper)
    # Calculation of the nearest obstacles
    def calculate_nearest_obstacles(self, state, obstacles):
        """
        Calculates the nearest obstacles w.r.t to the robot current state and obstacle position
        """
        robot_pos = state[0:2]
        nearest_obstacles_ind = np.where(
            np.linalg.norm(robot_pos - obstacles, axis=1) < self.obstacle_dist_tol)

        nearest_obstacles = obstacles[nearest_obstacles_ind]

        return nearest_obstacles

    # 'angle' heuristic
    def calc_obs_dist_heuristic(self, trajectory, nearest_obstacles):
        """
        Calculates the Obstacle Heuristic cost function (dist(v,omega) from DWA paper)
        """
        trajectory_positions = trajectory[:, 0:2]
        obs_x = nearest_obstacles[:,0]
        obs_y = nearest_obstacles[:,1]

        dx = trajectory_positions[:, 0] - obs_x[:, None]
        dy = trajectory_positions[:, 1] - obs_y[:, None]
        euclidean_dist = np.hypot(dx, dy)

        if np.array(euclidean_dist <= self.robot.robot_radius + 0.2).any():
            return np.inf
        elif euclidean_dist.size == 0:
            # No nearest osbtacle therefore return cost of 0
            return 0.0
        else:
            # Return the inverse since we are trying to maximize the objective func for obstacles
            # Smaller the dist the higher the robot's desire to move around it
            min_dist = np.min(euclidean_dist)
            return 1.0 / min_dist

    # 'dist' heuristic
    def calc_goal_heuristic(self, trajectory, goal_pose):
        """
        Calculates the goal heuristic cost function (heading(v,omega) from DWA paper)
        """
        ### Extract positions
        goal_pos = goal_pose[0:2]
        traj_pos = trajectory[-1, 0:2]  # Only considering end trajectory for goal heuristic
        
        ### Calculation of angle between goal and trajectory vectors
        goal_unit_vec = goal_pos / (np.linalg.norm(goal_pos) +  np.finfo(np.float32).eps)   # Add eps to avoid division by 0
        traj_unit_vec = traj_pos / (np.linalg.norm(traj_pos) +  np.finfo(np.float32).eps)   # Add eps to avoid division by 0
        traj_dot_goal = np.dot(goal_unit_vec, traj_unit_vec)
        
        error_angle = np.arccos(traj_dot_goal) 

        ### Euclidean cost
        euclidean_dist_cost = np.linalg.norm(goal_pos - traj_pos)
        
        ### Total cost
        cost = error_angle + euclidean_dist_cost

        return cost

    # 'vel' heuristic
    def calc_vel_heuristic(self, trajectory, vel_ref):
        """
        Calculates the cost function for the speed to support fast movements (vel(v,omega) from DWA paper)
        """
        # We can just take the squared error between the desired maximum speed 
        # and the trajectory speed! It's like in control systems!
        # I.e the error is the cost!
        vel_error = vel_ref - trajectory[-1,3]

        return np.abs(vel_error)


