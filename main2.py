import operator
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs, Debug_Msg
import time
import numpy as np
import math
from rrt import RRT
import DWA

import random
'''
def get_obstacle_list(vision) :
	robot_list = []
	for i in range(16) :
		robot_list.append(vision.yellow_robot[i]) 
	obstacle_list = [(robot.x,robot.y,180) for robot in robot_list]
	return obstacle_list
这个函数修改了，保存一下
'''


def obstacle_change(vision,last_obstacle_list) :
	new_obstacle_list = get_obstacle_list(vision)
	for i in range(len(new_obstacle_list)) :
		if not operator.eq(new_obstacle_list[i],last_obstacle_list[i]) :
			return True
	return False

def get_obstacle_list(vision) :
	robot_list = []
	for i in range(16) :
		robot_list.append(vision.yellow_robot[i]) 
	obstacle_list = [(robot.x,robot.y,180,0 if abs(robot.raw_vel_x) < 0.0001 else robot.raw_vel_x,0 if abs(robot.raw_vel_y) < 0.0001 else robot.raw_vel_y) for robot in robot_list]
	return obstacle_list
def find_way(rand_area,obstacle_list,start,goal) :
	path = None
	while path is None :
		rrt = RRT(rand_area=rand_area, obstacle_list=obstacle_list, expand_step=0.07, goal_sample_rate=15, max_iter=2000,max_err=0.02)
		path = rrt.route_plan(start=start, goal=goal)
	return path,rrt


if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	rate = 1000.0
	#enviroment
	start = np.array([2400, 1500])/ rate
	goal = np.array([-2400, -1500]) / rate
	rand_area = [[-4500/rate, -3000/rate], [4500/rate, 3000/rate]]
	obstacle_list = np.array(get_obstacle_list(vision)) / rate
	#trajPlanning
	forward_end = False
	direct = 1
	update_dist = 0.1
	config = DWA.DWAConfig()
	controller = DWA.DWA(config=config)
	v, w = 0, 0
	iter = 0
	goal_point_index = 0
	mypath = None
	rrt = None
	traj = []
	time.sleep(1)
	while True:
		robot_x = vision.my_robot.x / rate
		robot_y = vision.my_robot.y / rate
		robot_orientation = vision.my_robot.orientation
		# v = vision.my_robot.vel_x / rate
		# w = vision.my_robot.raw_orientation
		#print("robot x :{:.2f}, y{:.2f}, orientation{:.2f}, v{:.2f}, w:{:.2f}".format(robot_x, robot_y, robot_orientation, v, w))
		if iter == 0 :
			iter += 1
			obstacle_list = np.array(get_obstacle_list(vision)) / rate
			print(obstacle_list)
			mypath, rrt = find_way(rand_area,obstacle_list,start,goal)
			print(mypath)

			print("plan Ok!")
			goal_point_index = len(mypath) - 4
			goal = mypath[goal_point_index]
		else :
			state = np.array([robot_x, robot_y, robot_orientation, v, w])
			obstacle_list = np.array(get_obstacle_list(vision)) / rate
			# print(obstacle_list)
			ctrl, traj, _, _ = controller.dwa_control(state,goal,obstacle_list)
			v = ctrl[0] 
			w = ctrl[1]
		# 2. send command
		# if math.hypot(robot_x - goal[0],robot_y - goal[1]) < update_dist :
		# 	if goal_point_index > 0 :
		# 		goal_point_index -= 1
		# 		goal = mypath[goal_point_index]
		# 	elif goal_point_index == 0 :
		# 		goal = mypath[goal_point_index]
		min_distance = (float('inf'),0)
		distance_to_choice = math.hypot(robot_x - goal[0],robot_y - goal[1])
		ratio_to_further = 1.2
		for i in range(len(mypath)):
			if min_distance[0] > math.hypot(robot_x - mypath[i][0],robot_y - mypath[i][1]):
				min_distance = (math.hypot(robot_x - mypath[i][0],robot_y - mypath[i][1]),i)
		if (min_distance[0] < distance_to_choice * ratio_to_further
				and min_distance[1] < goal_point_index) :
			goal_point_index = min_distance[1]
			goal = mypath[goal_point_index]
		if goal_point_index >0 and random.random() < 0.1:
			goal_point_index -= 1
		if distance_to_choice < 0.8 :	#步长70-》0.07
			if goal_point_index > 0 :
				goal_point_index -= 1
				goal = mypath[goal_point_index]
			elif goal_point_index == 0 :
				goal = mypath[goal_point_index]
				if math.hypot(vision.my_robot.x--2400,vision.my_robot.y--1500)<50 or math.hypot(vision.my_robot.x-2400,vision.my_robot.y-1500)<50:
					mypath.reverse()
					goal_point_index = len(mypath) - 4
					goal = mypath[goal_point_index]
			#print("It's here\n\n\Change index")

		action.sendCommand(vx=v*rate, vy=0, vw=w)
		# 3. draw debug msg
		package = Debug_Msgs()
		#debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
		if mypath is not None :
			node_list = rrt.get_tree()
			for node in node_list :
				x1 = node.x * rate
				y1 = node.y * rate
				if node.parent is not None :
					x2 = node_list[node.parent].x * rate
					y2 = node_list[node.parent].y * rate
					debugger.draw_line(package,x1,y1,x2,y2,color=Debug_Msg.WHITE)
			for i in range(len(mypath) - 1):
				debugger.draw_line(package,mypath[i][0]*rate,mypath[i][1]*rate,mypath[i+1][0]*rate,mypath[i+1][1]*rate,color=Debug_Msg.GREEN)
		for i in range(len(traj) - 1):
			debugger.draw_line(package, traj[i,0]*rate, traj[i,1]*rate, traj[i+1,0]*rate, traj[i+1,1]*rate, color=Debug_Msg.RED)
		debugger.draw_circle(package, goal[0] * rate, goal[1] * rate)
		estimate_time = 5
		estimate_step = 0.3
		for item in obstacle_list:
			if len(item>3):
				for i in range(estimate_time):
					debugger.draw_circle(package, item[0]*rate+item[3]*i*rate*estimate_step, item[1] * rate++item[4]*i*rate*estimate_step)
		debugger.send(package)

		time.sleep(0.01)

