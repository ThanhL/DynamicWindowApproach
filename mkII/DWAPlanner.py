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

        # Distance tolerance to obstacle
        self.obstacle_dist_tol = 4

    ### Dynamic window calculations
    # Space of possible velocities
    def calculate_Vs(self):
        return [self.robot.min_vel, self.robot.max_vel, self.robot.min_omega, self.robot.max_omega]

    # Dynamic windows
    def calculate_Vd(self, robot_state):
        robot_x, robot_y, robot_theta, v_a, omega_a = robot_state
        return [v_a - self.robot.max_acc * self.dt, v_a + self.robot.max_acc * self.dt,               # Linear velocity
            omega_a - self.robot.max_ang_acc * self.dt, omega_a + self.robot.max_ang_acc * self.dt]   # Angular velocity

    # Admissiable velocites
    def is_admissable(self, trajectory, control_input, obstacles):
        ### Control input
        v, omega = control_input

        ### Calculate distance
        print(obstacles.shape)
        obs_x, obs_y = obstacles[:,0], obstacles[:,1]

        delta_x = trajectory[:, 0] - obs_x[:, None]
        delta_y = trajectory[:, 1] - obs_y[:, None]

        dist = np.sqrt(np.square(delta_x) + np.square(delta_y))

        ### Calculation if admissable
        v_admissable_criteria = np.sqrt(2 * dist * self.robot.max_acc)
        omega_admissable_criteria = np.sqrt(2 * dist * self.robot.max_ang_acc)

        if np.array(v <= v_admissable_criteria).any() and np.array(omega <= omega_admissable_criteria).any():
            return True
        else:
            return False

    # Calculate resulting search space windows
    def calculate_Vr(self, robot_state):
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
        x_state = deepcopy(x)
        trajectory = x_state

        for i in range(self.n_horizon):
            x_state = self.robot.motion_model(x_state, u, self.dt)
            trajectory = np.vstack((trajectory, x_state))

        return trajectory


    def calc_dwa_control(self, robot_state, robot_goal, obstacles):
        """
        DWA control inputs calculation
        """
        # Best Metrics Initializer
        minimum_cost = np.inf       # Initialize minimum cost to extremely large initially
        best_control_input = np.zeros(2)    # Control input 
        best_trajectory = deepcopy(robot_state)

        # Compute the resulting search space
        Vr_v, Vr_omega = self.calculate_Vr(robot_state)

        # Trajectory set (store all possible circular trajectories for visualization)
        num_possible_trajectories = Vr_v.shape[0] * Vr_omega.shape[0]
        trajectory_set = np.zeros((0, self.n_horizon+1, robot_state.shape[0]))

        # Evaluate 
        x_init = deepcopy(robot_state)
        for v in Vr_v:
            for omega in Vr_omega:
                ### Generate predicted trajectories with (v, omega) pair from search space
                control_input = np.array([v, omega])
                trajectory = self.generate_trajectory(x_init, control_input)

                ### Evaluate the paths for nearest obstacles
                nearest_obstacles = self.calculate_nearest_obstacles(robot_state, obstacles)

                ### Check if velocity is admissable
                if True or self.is_admissable(trajectory, control_input, nearest_obstacles):
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

                        ### Prevention of getting stuck in (v,omega) = 0 search space
                        if (abs(control_input[0]) < 0.001 and abs(robot_state[3]) < 0.001):
                            control_input[0] = -self.robot.max_omega

        print("best_control_input: ", best_control_input)
        print("minimum_cost: ", minimum_cost)
        print("Vr_v: ", Vr_v)
        print("Vr_omega: ", Vr_omega)
        print("num_possible_trajectories: ", num_possible_trajectories)
        print("robot_state_shape: ", robot_state.shape)
        print("trajectory_set_shape: ", trajectory_set.shape)


        return best_control_input, best_trajectory, trajectory_set

    ### Heuristics (from DWA paper)
    # Calculation of the nearest obstacles
    def calculate_nearest_obstacles(self, state, obstacles):
        nearest_obstacles = []
        robot_pos = state[0:2]

        for obs in obstacles:
            # Distance from robot pose to obstacle
            euclidean_dist_to_obs = np.linalg.norm(robot_pos - obs)

            # If robot is within range of the obstacle
            if euclidean_dist_to_obs < self.obstacle_dist_tol:
                nearest_obstacles.append(obs)

        return np.array(nearest_obstacles)

    # 'angle' heuristic
    def calc_obs_dist_heuristic(self, trajectory, nearest_obstacles):
        cost_obs = 0.0
        min_dist = np.inf   # Minimum distance to obstacle

        # For every trajectory point 
        for i in range(trajectory.shape[0]):
            for obs in nearest_obstacles:
                # Extract the trajectory's position
                trajectory_pos = trajectory[i, 0:2]

                # Calculate the distance between the obstacle and the trajectory's position
                euclidean_dist_to_obs = np.linalg.norm(trajectory_pos - obs)

                # Collision with obstacle? 
                # Note if dist between robot position and obstacle point is less than the radius of the robot
                # Then the obstacle point is within the robot (i.e collision)


                if euclidean_dist_to_obs <= self.robot.robot_radius:
                    return np.inf

                if euclidean_dist_to_obs < min_dist:
                    min_dist = euclidean_dist_to_obs

        # Return the inverse since we are trying to maximize the objective func for obstacles
        # Smaller the dist the higher the robot's desire to move around it
        return 1.0 / min_dist

    # 'dist' heuristic
    def calc_goal_heuristic(self, trajectory, goal_pose):
        ### Calculation of euclidean heuristic
        goal_pose_x, goal_pose_y, goal_pose_theta = goal_pose
        traj_end_x, traj_end_y, traj_end_theta, _, _ = trajectory[-1]

        # Magnitude calculations
        goal_mag = np.sqrt(goal_pose_x**2 + goal_pose_y**2)
        traj_end_mag = np.sqrt(traj_end_x**2 + traj_end_y**2)

        # Delta
        delta_x = goal_pose_x - traj_end_x
        delta_y = goal_pose_y - traj_end_y

        # Dot product between trajectory end and goal pose
        dot_product = (goal_pose_x * traj_end_x) + (goal_pose_y * traj_end_y)
        error_cos_theta = dot_product / (goal_mag * traj_end_mag + np.finfo(np.float32).eps)
        error_angle = np.arccos(error_cos_theta) 


        # ### Orientation error
        # error_angle = np.arctan2(delta_y, delta_x) - traj_end_theta

        ### Euclidean istance
        euclidean_dist_cost = np.sqrt((goal_pose_x - traj_end_x)**2 + (goal_pose_y - traj_end_y)**2)

        cost = error_angle + euclidean_dist_cost

        return cost

    # 'vel' heuristic
    def calc_vel_heuristic(self, trajectory, vel_ref):
        ### Calculate the cost for speed
        # We can just take the squared error between the desired maximum speed 
        # and the trajectory speed! It's like in control systems!
        # I.e the error is the cost!
        vel_error = vel_ref - trajectory[-1,3]


        # MSE
        # vel_error =  0.5 * (vel_error)**2

        return np.abs(vel_error)



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
