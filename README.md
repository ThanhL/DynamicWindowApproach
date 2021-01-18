# DynamicWindowApproach
Dynamic Window Approach Assignment


## Setup/Installation

This project was built and tested with Python 3.8.5. The required packages and their versions are located in the requirements.txt file. 

To run this project, first clone the repository and install the required python packages with the requirements.txt:

```
$ cd <directory you want to install to>
$ git clone https://github.com/ThanhL/DynamicWindowApproach.git
$ cd DynamicWindowApproach
$ pip install -r requirements.txt
```

## Usage

### How to run
The crux of the code is ran through `driver.py`. To run the dwa simulation, we run the following command:

```
python driver.py --start_position start_x start_y start_theta --goal_position goal_x goal_y goal_theta
``` 

where,
* start_x, start_y, start_theta: is the starting position in x,y (in m) and starting orientation (in radians) respectively.
* goal_x, goal_y, goal_theta: is the goal position in x,y (in m) and goal orientation (in radians) respectively.

### Custom Configuration Files
Note that this code supports custom user-input configurations for the simulation. The default configuration files are located in the config folder. There are 3 configuration files required for the simulation:

* planner_config.yaml: config file containing all the parameters for the dynamic window approach planner.
* robot_config.yaml: config file containing all the parameters for the robot specifications.
* sim_config.yaml: config file containing all the parameters for the simulation.

Note that custom configuration files can also be passed via the command line:

```
python driver.py --start_position start_x start_y start_theta --goal_position goal_x goal_y goal_theta --robot_config <robot_config_file> --planner_config <planner_config_file> --sim_config <sim_config_file>
``` 
where `robot_config_file, planner_config_file, sim_config_file` are the string of the configuration files (with their full path included).


## References
[1] Thrun, Sebastian., Burgard, Wolfram., Fox, Dieter. *The Dynamic Window Approach to Collision Avoidance*, 1997.

[2] Control of Mobile Robots- 2.2 Differential Drive Robots, Georgia Tech University. https://www.youtube.com/watch?v=aE7RQNhwnPQ
