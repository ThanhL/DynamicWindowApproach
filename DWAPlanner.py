import numpy as np
import matplotlib.pylplot as plt

class DWAPlanner():
	def __init__(self, planner_config):


		self.alpha = planner_config["alpha"]		# Goal Gain heuristic
		self.beta = planner_config["beta"]			# Distance to Obstacle Gain heuristic
		self.gamma = planner_config["gamma"]		# Forward Velocity Gain Heuristic
		return


	def calc_dyn_win(self):
		return

	def calc_dwa_control(robot_pose, robot_goal):
		return

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