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

    def is_admissable(self, trajectory, control_input, obstacles):
        ### Control input
        v, omega = control_input

        ### Calculate distance
        obs_x, obs_y = obstacles[0:2]

        delta_x = trajectory[:, 0] - obs_x[:, None]
        delta_y = trajectory[:, 1] - obs_y[:, None]

        dist = np.sqrt(np.square(delta_x) + np.square(delta_y))



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


        # print("Vs: ", Vs)
        # print("Vd: ", Vd)
        # print("Vr: ", Vr)

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


    def calc_dwa_control(self, robot_pose, robot_goal, obstacles):
        """
        DWA control inputs calculation
        """
        # Best Metrics Initializer
        minimum_cost = np.inf       # Initialize minimum cost to extremely large initially
        best_control_input = np.zeros(2)    # Control input 
        best_trajectory = deepcopy(robot_pose)

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


                ### Cost calculation
                angle_cost = self.alpha * self.calc_goal_heuristic(trajectory, robot_goal)
                dist_cost = self.beta * self.calc_obs_dist_heuristic(trajectory, obstacles)
                vel_cost = self.gamma * self.calc_vel_heuristic(trajectory, self.max_vel)
                
                # Total cost
                total_cost = angle_cost + dist_cost + vel_cost


                # ## Debugging
                # print("(v,r): ({:.3f}, {:.3f})".format(v, omega))
                # print("robot pose: ", x_init)
                # # print("traj: ", trajectory)

                # print("angle_cost: {:.4f}".format(angle_cost))
                # print("dist_cost: {:.4f}".format(dist_cost))
                # print("vel_cost: {:.4f}".format(vel_cost))
                # print()

                ### Update best costs & store the best control inputs + trajectory
                if minimum_cost >= total_cost:
                    # print("[!] Best Found (v,w): ({:.3f}, {:.3f})".format(v, omega))

                    minimum_cost = total_cost
                    best_control_input[:] = control_input
                    best_trajectory = trajectory

        print("best_control_input: ", best_control_input)
        print("minimum_cost: ", minimum_cost)
        print("Vr_v: ", Vr_v)
        print("Vr_omega: ", Vr_omega)
        print("num_possible_trajectories: ", num_possible_trajectories)
        print("robot_pose_shape: ", robot_pose.shape)
        print("trajectory_set_shape: ", trajectory_set.shape)


        return best_control_input, best_trajectory, trajectory_set

    ### Heuristics (from DWA paper)
    # 'angle' heuristic
    def calc_obs_dist_heuristic(self, trajectory, obstacles):
        obs_x, obs_y = obstacles[0:2]

        delta_x = trajectory[:, 0] - obs_x[:, None]
        delta_y = trajectory[:, 1] - obs_y[:, None]

        dist = np.sqrt(np.square(delta_x) + np.square(delta_y))



        # print("---- Obstaccle Distance heuristic ----")
        # print(obstacles.shape)
        # print(trajectory[:, 0:2].shape)

        # print("traj_pos: ", traj_pos)
        # print("delta_x: ", delta_x)
        # print("delta_y: ", delta_y)


        if self.robot_type == "circle":
            if np.array(dist <= self.robot_radius).any():
                # This trajectory collides with an obstacle!
                return np.inf

        mininum_dist = np.min(dist)

        # Return the inverse since we are trying to maximize the objective func for obstacles
        # Smaller the dist the higher the robot's desire to move around it
        return 1.0 / mininum_dist

    # 'dist' heuristic
    def calc_goal_heuristic(self, trajectory, goal_pose):
        ### Calculation of euclidean heuristic
        goal_pose_x, goal_pose_y, goal_pose_theta, _, _ = goal_pose
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

        # error_angle = np.arctan2(delta_x, delta_y)
        # cost_angle = error_angle - traj_end_theta
        # cost = np.abs(np.arctan2(np.sin(cost_angle), np.cos(cost_angle)))
        
        # print("dot_product: ", dot_product)
        # print("goal_mag: ", goal_mag)
        # print("traj_end_mag: ", traj_end_mag)

        # Return error angle as the cost
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
