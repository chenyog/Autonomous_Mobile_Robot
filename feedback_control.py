import numpy as np
import math
# Feedback Control for trajactory planning on a plane by lei chengyong

class feedback_control :
    """初始化
    feedback_control 分为简易反馈控制和改进反馈控制两种方法
    """
    def __init__(self, 
                 k1 = 1.0, 
                 k2 = 5.0,
                 mu = 0.1, 
                 lamda = 2.0,
                 k_rho = 3.0,
                 k_alpha = 8.0,
                 k_beta = 1.5,
                 v_max = 30.0, #转换为cm/s
                 w_max = np.pi,
                 max_err = 1, #转换为cm
                 period = 0.01):
        
        #advanced feed_back_control parameter
        self.k1 = k1
        self.k2 = k2
        self.mu = mu
        self.lamda = lamda

        #simple feed_back_control parameter
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta

        #约束条件和控制周期
        self.v_max = v_max
        self.w_max = w_max
        self.max_err = max_err
        self.period = period

    @staticmethod
    def limit(theta) :
        while theta > np.pi :
            theta -= 2*np.pi
        while theta <= -np.pi :
            theta += 2*np.pi
        return theta
    
    #此函数未完善
    def calc_vw_simple(self,start_pose,goal_pose) : 
        start_x = start_pose[0]
        start_y = start_pose[1]
        start_theta = start_pose[2]

        goal_x = goal_pose[0]
        goal_y = goal_pose[1]
        goal_theta = goal_pose[2]
        
        theta1 = math.atan2(goal_y - start_y, goal_x - start_x)
        theta2 = goal_theta
        beta = self.limit(theta1 - theta2)
        rho = math.sqrt((goal_y - start_y)**2 + (goal_x - start_x)**2)
        if rho < self.max_err :
            return 0,0
        theta = self.limit(start_theta - goal_theta)
        alpha = self.limit(beta - theta)
        
        if alpha <= np.pi/2 and alpha > -np.pi/2 :
            v = self.k_rho * rho
            w = self.k_alpha*alpha + self.k_beta * beta
        else:
            v = self.k_rho * rho
            w = self.k_alpha*alpha + self.k_beta * beta
        
        return v,w
           
    def calc_vw_advanced(self,start_pose,goal_pose) : 
        """改进反馈控制，根据当前位姿和目标位姿计算（v,w)
        
        Args:
            start_pose (_type_): 当前时刻位姿
            goal_pose (_type_): 目标位姿

        Returns：
            v,w 

        """
        start_x = start_pose[0]
        start_y = start_pose[1]
        start_theta = start_pose[2]

        goal_x = goal_pose[0]
        goal_y = goal_pose[1]
        goal_theta = goal_pose[2]

        theta1 = math.atan2(goal_y - start_y, goal_x - start_x)
        theta2 = goal_theta
        beta = self.limit(theta1 - theta2)
        rho = math.sqrt((goal_y - start_y)**2 + (goal_x - start_x)**2)
        if rho < self.max_err :
            return 0,0
        theta = self.limit(start_theta - goal_theta)
        alpha = self.limit(beta - theta)

        kappa = 1/rho * (self.k2 * (alpha - math.atan(-self.k1 * beta)) + (1 + self.k1/(1+(self.k1*beta)**2)) * math.sin(alpha))
        v = self.v_max / (1 + self.mu * math.pow(math.fabs(kappa),self.lamda))
        w = v * kappa 
        return v,w
        

