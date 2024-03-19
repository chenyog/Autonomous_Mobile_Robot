import operator
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs, Debug_Msg
import time
import numpy as np
import math
if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	time.sleep(1)
	while True:
		# action.sendCommand(vx=100, vy=0, vw=0)
		action.controlObs(vision)
		package = Debug_Msgs()
		debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
		debugger.send(package)
		time.sleep(0.01)