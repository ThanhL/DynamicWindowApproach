import numpy as np

class Map():
    def __init__(self, min_x=-10, max_x=10, min_y=-10, max_y=10, num_obstacles=10):
        ### Constructor variables
        self.map_size_x = 100
        self.map_size_y = 100
        
        ### Obstacles generation
        self.obstacles = self.generate_obstacles(num_obstacles, min_x, max_x, min_y, max_y)

    def generate_obstacles(self, num_obstacles, min_x, max_x, min_y, max_y):
        ### Obstacle generation function
        obs_x = np.random.uniform(low=min_x, high=max_x, size=(num_obstacles,))
        obs_y = np.random.uniform(low=min_y, high=max_y, size=(num_obstacles,))
        obs = np.array([obs_x, obs_y]).T

        return obs



if __name__ == "__main__":
    ### Testin map
    sim_map = Map()

    print("map obstacles (randomly generated): \n", sim_map.obstacles)
