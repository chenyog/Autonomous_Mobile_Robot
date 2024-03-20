import math
import time

import numpy as np

def limit(theta) :
    while theta > np.pi :
        theta -= 2*np.pi
    while theta <= -np.pi :
        theta += 2*np.pi
    return theta

def KinematicsModel(state,vel,dt) :
    #print("state is "+str(state))
    state_predict = np.empty(5)
    theta = state[2]
    v = vel[0]
    w = vel[1]
    # state_predict[0] = state[0] + v * dt * math.cos(theta) 
    # state_predict[1] = state[1] + v * dt * math.sin(theta)
    if not (abs(w) <= 0.00001):
        state_predict[0] = state[0] - (v / w)*math.sin(theta) + (v / w) * math.sin(theta+w*dt)
        state_predict[1] = state[1] + (v / w)*math.cos(theta) - (v / w) * math.cos(theta+w*dt)
    else:
        state_predict[0] = state[0] + v*math.cos(theta)*dt
        state_predict[1] = state[1] + v*math.sin(theta)*dt
    state_predict[2] = limit(theta + w*dt)
    state_predict[3] = v
    state_predict[4] = w
    return state_predict

def SimplifiedKinematicsModel(state,vel,dt):
    state_predict = np.empty(5)
    theta = state[2]
    v = vel[0]
    w = vel[1]
    if not (abs(w) <= 0.00001):
        state_predict[0] = state[0] - (v / w) * math.sin(theta) + (v / w) * math.sin(theta + w * dt)
        state_predict[1] = state[1] + (v / w) * math.cos(theta) - (v / w) * math.cos(theta + w * dt)
    else:
        state_predict[0] = state[0] + v * math.cos(theta) * dt
        state_predict[1] = state[1] + v * math.sin(theta) * dt
    return state_predict

class DWAConfig:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter

        self.v_max =  1.0 # [m/s]
        self.v_min = -1.0  # [m/s]

        self.w_max = 0.5 * math.pi  # [rad/s]
        self.w_min = -0.5 * math.pi  # [rad/s]

        self.a_vmax = 1.0 # [m/ss]
        self.a_wmax = math.pi # [rad/ss]

        self.dt = 0.1  # [s] Time tick for motion prediction
  
        self.v_sample = self.a_vmax * self.dt / 10.0  # [m/s]
        self.w_sample = self.a_wmax * self.dt / 10.0  # [rad/s]

        self.predict_time = 1  # [s]

        self.heading_ratio = 1.0
        self.close_ratio = 1.0*50
        self.vel_ratio = 0.7*5
        self.obs_ratio = 1.0*1

        self.min_dist = 0.0
        
class DWA:
    def __init__(self,config) :
        self.dt=config.dt
        self.v_min=config.v_min
        self.w_min=config.w_min
        self.v_max=config.v_max 
        self.w_max=config.w_max
        self.predict_time = config.predict_time
        self.a_vmax = config.a_vmax
        self.a_wmax = config.a_wmax
        self.v_sample = config.v_sample # 
        self.w_sample = config.w_sample #
        self.heading_ratio = config.heading_ratio #
        self.close_ratio= config.close_ratio #
        self.vel_ratio = config.vel_ratio #
        self.obs_ratio = config.obs_ratio #
        self.min_dist = config.min_dist #

    def dwa_control(self,state,goal,obstacle_list  = None, radius = None) :
        if obstacle_list is not None:
            if radius is not None :  #
                obstacle =  np.zeros(shape=(np.shape(obstacle_list)[0],3))
                obstacle[:,0] = obstacle_list[:,0]
                obstacle[:,1] = obstacle_list[:,1]
                obstacle[:,2] = radius
            else :
                obstacle = obstacle_list
        else :
            print("DWA : no obstacle")
            return -1
        return self.traj_evaluation(state,goal,obstacle)
    
    def cal_vel_limit(self):
        return [self.v_min,self.v_max,self.w_min,self.w_max]
    
    def cal_accel_limit(self,v,w):
        v_low = v - self.a_vmax*self.dt
        v_high = v + self.a_vmax*self.dt
        w_low = w - self.a_wmax*self.dt
        w_high = w + self.a_wmax*self.dt
        return [v_low, v_high,w_low, w_high]
    
    def cal_obstacle_limit(self,state,obstacle) :
        v_high = np.sqrt(2*self._dist(state,obstacle)*self.a_vmax)
        v_low = -v_high
        w_high = np.sqrt(2*self._dist(state,obstacle)*self.a_wmax)
        w_low = -w_high
        return [v_low,v_high,w_low,w_high]
    
    def _dist(self,state,obstacle) :
        ox = obstacle[:,0]
        oy = obstacle[:,1]
        dx = state[0,None] - ox[:,None] #从行变列
        dy = state[1,None] - oy[:,None]
        r = np.hypot(dx,dy)
        r[:,0] -= obstacle[:,2]   #缩距离
        return np.min(r)
          
    def calc_vel_space_window(self,v,w,state,obstacle):
        Vm = self.cal_vel_limit()   #速度：小大小大
        Vd = self.cal_accel_limit(v,w)  #可达速度小大小大
        Va = self.cal_obstacle_limit(state,obstacle)    #
        a = max([Vm[0], Vd[0], Va[0]])
        b = min([Vm[1], Vd[1], Va[1]])
        c = max([Vm[2], Vd[2], Va[2]])
        d = min([Vm[3], Vd[3], Va[3]])
        return [a,b,c,d]
    
    def traj_predict(self,cur_state,v,w):
        cur_state = np.array(cur_state)
        traj = cur_state
        time = 0
        while time <= self.predict_time :
            cur_state = KinematicsModel(cur_state,[v,w],self.dt)
            traj = np.vstack((traj,cur_state))
            time += self.dt
        return traj
    
    def normalize(self,data):
        _range = np.max(data) - np.min(data)
        normal = (data[None,:] - np.min(data)) / _range
        return normal
    
    def traj_evaluation(self,state,goal,obstacle):
        G_min = float('inf')
        dynamic_window_vel = self.calc_vel_space_window(state[3],state[4], state, obstacle)
        evaluation = []
        best_vw = np.array([0, 0])
        traj = []
        best_traj = []
        for v in np.arange(dynamic_window_vel[0],dynamic_window_vel[1],self.v_sample):
            for w in np.arange(dynamic_window_vel[2], dynamic_window_vel[3], self.w_sample):   
                trajectory = self.traj_predict(state, v, w)  
                # heading_eval = self.heading_ratio*self.heading(trajectory,goal)
                close_eval = self.close_ratio*self.close(trajectory,goal)
                vel_eval = self.vel_ratio*self.vel(trajectory)
                obs_eval = self.obs_ratio*self.dist(trajectory,obstacle)
                cur_eval =  close_eval + vel_eval + obs_eval
                # cur_eval = heading_eval + close_eval + vel_eval + obs_eval
                evaluation.append(cur_eval)
                if cur_eval < G_min :
                    G_min = cur_eval
                    best_vw = np.array([v,w])
                    best_traj = trajectory
                    #print(best_vw)
                if G_min == float('inf'):
                    print("die of all G are die")
                    if(state[3]<100.0):
                        i = 0
                        index = 0
                        min_dis = float('inf')
                        for obs in obstacle:
                            ox_t = obs[0]
                            oy_t = obs[1]
                            rx = state[0]
                            ry = state[1]
                            dis_t = np.hypot(ox_t-rx, oy_t-ry)
                            if dis_t < min_dis:
                                min_dis = dis_t
                                index = i
                            i = i+1
                        forward_FLAG = False
                        turning_clock_FLAG = False#顺时针，w>0
                        if np.hypot(obstacle[index][0]-SimplifiedKinematicsModel(state,[0.00003,0.0],0.01)[0],obstacle[index][1] - SimplifiedKinematicsModel(state,[0.00003,0.0],0.01)[1])>min_dis:
                            forward_FLAG = True ##上面最好可以结合上角度，让逃逸更快
                        if forward_FLAG:
                            if np.hypot(obstacle[index][0]-SimplifiedKinematicsModel(state,[1.0,0.5],0.01)[0],obstacle[index][1] - SimplifiedKinematicsModel(state,[1.0,0.5],0.01)[1])>np.hypot(obstacle[index][0]-SimplifiedKinematicsModel(state,[1.0,0.5],0.01)[0],obstacle[index][1] - SimplifiedKinematicsModel(state,[1.0,0.5],0.01)[1]):
                                turning_clock_FLAG = True
                        else:
                            if np.hypot(obstacle[index][0]-SimplifiedKinematicsModel(state,[-1.0,0.5],0.01)[0], obstacle[index][1] - SimplifiedKinematicsModel(state,[-1.0,0.5],0.01)[1])>np.hypot(obstacle[index][0]-SimplifiedKinematicsModel(state,[-1.0,0.5],0.01)[0],obstacle[index][1] - SimplifiedKinematicsModel(state,[-1.0,0.5],0.01)[1]):
                                turning_clock_FLAG = True
                        print(forward_FLAG)
                        print(min_dis)
                        if forward_FLAG:
                            if turning_clock_FLAG:
                                best_vw = np.array([1.0, 0.5])
                                best_traj = self.traj_predict(state, 1.0, 0.5)
                            else:
                                best_vw = np.array([1.0, -0.5])
                                best_traj = self.traj_predict(state, 1.0, -0.5)
                        else:
                            if turning_clock_FLAG:
                                best_vw = np.array([-1.0, 0.5])
                                best_traj = self.traj_predict(state, -1.0, 0.5)
                            else:
                                best_vw = np.array([-1.0, -0.5])
                                best_traj = self.traj_predict(state, -1.0, -0.5)
                    else:
                        best_vw = np.array([-state[3],state[4]])
                        best_traj = self.traj_predict(state, -state[3], state[4])

                    #exit()
                evaluation.append(cur_eval)
                traj.append(trajectory)
        evaluation = np.array(evaluation)
        return best_vw, best_traj, traj, evaluation

        
    def heading(self,traj,goal):
        dx = goal[0] - traj[-1, 0]
        dy = goal[1] - traj[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = limit(error_angle - traj[-1, 2])
        cost = abs(cost_angle)
        return cost
    
    def close(self,traj,goal):
        dx = goal[0] - traj[-1, 0]
        dy = goal[1] - traj[-1, 1]
        return np.sqrt(dx**2 + dy**2)
    
    def vel(self, traj):
        return self.v_max - traj[-1,3]

    def dist(self,traj,obstacle) :  #12*5,16*5

        FLAG_INCLUDE_VEL = False
        for item in obstacle:
            if len(item) < 5:
                break
            else:
                FLAG_INCLUDE_VEL = True
                break
        if not FLAG_INCLUDE_VEL:
            ox = obstacle[:, 0]
            oy = obstacle[:, 1]
            d = obstacle[:, 2]
            dx = traj[:, 0] - ox[:, None]#16*12
            dy = traj[:, 1] - oy[:, None]
            r = np.hypot(dx, dy)
            r = r[:,None] - d[:,None]
            if np.array(r < self.min_dist).any() :
                return float('inf')
            else :
                return 1/np.min(r)
        else:
            ox = obstacle[:, 0]
            oy = obstacle[:, 1]
            d = obstacle[:, 2]
            vx = obstacle[:, 3]*1000*0.3#rate and estimate_step
            vy = obstacle[:, 4]*1000*0.3
            estimate_time = 2
            #print("ox is"+str(ox) + '\n')
            #print("vx is" + str(vx))
            #
            long_list_as_traj = np.ones_like(ox)
            # 初始化 dx 和 dy 数组
            dx = traj[:, 0] - ox[:, None]
            dy = traj[:, 1] - oy[:, None]

            for i in range(1 , estimate_time):
                new_dx = traj[:, 0] - (ox + i*vx)[:, None]  #方向是否需要调整
                new_dy = traj[:, 1] - (oy + i*vy)[:, None]
                dx = np.vstack((dx, new_dx))
                dy = np.vstack((dy, new_dy))

            r = np.hypot(dx, dy)
            r = r[:, None] - d[:, None]
            if np.array(r < self.min_dist).any():
                return float('inf')
            else:
                return 1 / np.min(r)

