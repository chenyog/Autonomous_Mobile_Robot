import operator
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs, Debug_Msg
import time
import numpy as np
import math
from rrt import RRT
from feedback_control import feedback_control as traj

def obstacle_change(vision,last_obstacle_list) :
	new_obstacle_list = get_obstacle_list(vision)
	for i in range(len(new_obstacle_list)) :
		if not operator.eq(new_obstacle_list[i],last_obstacle_list[i]) :
			return True
	return False

def get_obstacle_list(vision) :
	robot_list = []
	for i in range(1,8) :
		robot_list.append(vision.blue_robot[i])
	for i in range(8) :
		robot_list.append(vision.yellow_robot[i]) 
	obstacle_list = [(robot.x,robot.y,300) for robot in robot_list]
	return obstacle_list

def find_way(rand_area,obstacle_list,start,goal) :
	path = None
	while path is None :
		rrt = RRT(rand_area=rand_area, obstacle_list=obstacle_list, expand_step=70.0, goal_sample_rate=10, max_iter=5000,max_err=20.0)
		path = rrt.route_plan(start=start, goal=goal)
	return path,rrt


if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	start = [2400, 1500]
	goal = [-2400, -1500]
	rand_area = [[-4500, -3000], [4500, 3000]]
	obstacle_list = get_obstacle_list(vision)
	#trajPlanning
	mypath = None
	forward_end = False
	rrt = None
	direct = 1
	goal_point_index = 0
	max_err = 20.0 #20.0
	waittime = 0
	rate = 100
	controller = traj(	k1 = 1, #1
                		k2 = 1.1, #2
                		mu = 0.5, #1
                		lamda = 0.2, #2
						v_max= 700.0/rate, #600
						max_err=max_err/rate
						)
	time.sleep(1)
	while True:
		# 1. path planning & velocity planning
		# Do something
		if obstacle_change(vision, obstacle_list) :
			obstacle_list = get_obstacle_list(vision)
			mypath, rrt = find_way(rand_area,obstacle_list,start,goal)
			goal_point_index = len(mypath) - 1
			waittime = 100
			forward_end = False
		# 2. send command
		if mypath is not None and waittime < 0:
			robot_x = vision.my_robot.x
			robot_y = vision.my_robot.y
			robot_orientation = vision.my_robot.orientation
			goal_x = mypath[goal_point_index][0]
			goal_y = mypath[goal_point_index][1]
			err = math.hypot(robot_x - goal_x,robot_y - goal_y)
			#cur_segment = math.hypot(mypath[goal_point_index-1][0] - goal_x,mypath[goal_point_index-1][1] - goal_y)
			if err <= max_err:
				if goal_point_index != 0 :
					goal_point_index -= 1
					goal_x = mypath[goal_point_index][0]
					goal_y = mypath[goal_point_index][1]
					#cur_segment = math.hypot(mypath[goal_point_index-1][0] - goal_x,mypath[goal_point_index-1][1] - goal_y)
				else :
					forward_end = True
			if goal_point_index != 0 :
				next_goal_x = mypath[goal_point_index-1][0]
				next_goal_y = mypath[goal_point_index-1][1]
				goal_orientation = math.atan2(next_goal_y-goal_y,next_goal_x-goal_x) 
				#goal_orientation = math.atan2(next_goal_y-goal_y,next_goal_x-goal_x) * (1 - err / cur_segment) 
			else :			
				goal_orientation = math.atan2(mypath[0][1]-mypath[1][1],mypath[0][0]-mypath[1][0])
			pose1 = [robot_x/rate,robot_y/rate,robot_orientation]
			pose2 = [goal_x/rate,goal_y/rate,goal_orientation ]
			if forward_end :
				mypath.reverse()
				goal_point_index = len(mypath) - 2
				forward_end = False
			v, w = controller.calc_vw_advanced(start_pose = pose1,goal_pose = pose2)
			action.sendCommand(vx=v*rate, vy=0, vw=w)
		else:
			waittime -= 1
		# print(vision.my_robot.orientation)
		#print("myrobot.x is"+ str(vision.my_robot.x) + "myrobot.y is" + str(vision.my_robot.y))
		# 3. draw debug msg

		package = Debug_Msgs()
		debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
		if mypath is not None :
			node_list = rrt.get_tree()
			for node in node_list :
				x1 = node.x
				y1 = node.y
				if node.parent is not None :
					x2 = node_list[node.parent].x
					y2 = node_list[node.parent].y
					debugger.draw_line(package,x1,y1,x2,y2,color=Debug_Msg.WHITE)
			for i in range(len(mypath) - 1):
				debugger.draw_line(package,mypath[i][0],mypath[i][1],mypath[i+1][0],mypath[i+1][1],color=Debug_Msg.GREEN)
		debugger.send(package)

		time.sleep(0.01)
