import numpy as np

class Map():
    def __init__(self, start_pose, end_pose, min_x=-10, max_x=10, min_y=-10, max_y=10, num_obstacles=10):
        ### Constructor variables
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

        self.start_pose = start_pose
        self.end_pose = end_pose

        ### Obstacles generation
        self.obstacles = self.generate_obstacles(start_pose, end_pose, num_obstacles, 
                                                min_x, max_x, min_y, max_y)

    def generate_obstacles(self, num_obstacles, min_x, max_x, min_y, max_y):
        """
        Simple obstacle generation function.

        This function does not take into account start/end pose so it might generate am
        obstacle on top of the start/end pose
        """
        ### Randomly generate obstacles x,y coord with map considerations
        obs_x = np.random.uniform(low=min_x, high=max_x, size=(num_obstacles,))
        obs_y = np.random.uniform(low=min_y, high=max_y, size=(num_obstacles,))
        obs = np.array([obs_x, obs_y]).T

        return obs

    def generate_obstacles(self, start_pose, end_pose, num_obstacles, min_x, max_x, min_y, max_y,
                        clearance=0.8):
        """
        More complicated obstacle generation function

        Function takes into account start/end pose and regenerates obstacles if the obstacles
        fall to near these points.
        """
        ### Randomly generate obstacles x,y coord with map considerations
        obs_x = np.random.uniform(low=min_x, high=max_x, size=(num_obstacles,))
        obs_y = np.random.uniform(low=min_y, high=max_y, size=(num_obstacles,))
        obs = np.array([obs_x, obs_y]).T

        ### Regenerate obstacles if generated obstacle is too close to robot's start pose and end pose
        euclidean_dist_start_to_obs = np.linalg.norm(start_pose[0:2] - obs, axis=1)
        euclidean_dist_end_to_obs = np.linalg.norm(end_pose[0:2] - obs, axis=1)

        conflicting_start_obs_ind = np.where(euclidean_dist_start_to_obs < clearance)
        conflicting_end_obs_ind = np.where(euclidean_dist_end_to_obs < clearance)

        print("[!] Found conflicting obstacles, regenerating obstacles...")
        print("[!] conflicting obstacles with starting pose: \n", obs[conflicting_start_obs_ind])
        print("[!] conflicting obstacles with ending pose: \n", obs[conflicting_end_obs_ind])

        ## Regenerate new obstacles to satisfy criteria
        for i in zip(*conflicting_start_obs_ind):
            new_obs = self.regenerate_obstacle(start_pose, obs[i], min_x, max_x, min_y, max_y, clearance)
            obs[i] = new_obs

        for j in zip(*conflicting_end_obs_ind):
            new_obs = self.regenerate_obstacle(end_pose, obs[j], min_x, max_x, min_y, max_y, clearance)
            obs[j] = new_obs


        print("\n[+] Obstacles now satisfy criteria, ready for simulation!\n")
        return obs

    def regenerate_obstacle(self, pose, obstacle, min_x, max_x, min_y, max_y, clearance):
        """
        Regenerate new obstacle coordinates until they don't fall near the point
        """
        new_obs_x = np.random.uniform(low=min_x, high=max_x)
        new_obs_y = np.random.uniform(low=min_y, high=max_y)
        new_obs = np.array([new_obs_x, new_obs_y])

        # Regenerate new obstacle until obstacle fits criteria.
        # BEWARE: This may result in an infinite loop
        while  np.linalg.norm(pose[0:2] - new_obs) < clearance:
            new_obs_x = np.random.uniform(low=min_x, high=max_x)
            new_obs_y = np.random.uniform(low=min_y, high=max_y)
            new_obs = np.array([new_obs_x, new_obs_y])

        return new_obs

if __name__ == "__main__":
    ### Testin map
    sim_map = Map()

    print("map obstacles (randomly generated): \n", sim_map.obstacles)
