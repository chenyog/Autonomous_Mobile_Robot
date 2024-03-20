import random
import math
import copy
import time
import numpy as np


class RRT:
    """初始化
    RRT* 方法
    """
    def __init__(self,
                 obstacle_list,         # 障碍物
                 rand_area,             # 采样的区域
                 expand_step=50.0,      # 步长
                 goal_sample_rate=10,   # 目标采样率
                 max_iter=200,          # 最大迭代次数
                 max_err = 10.0):       # 目标误差容限 

        self.start = None
        self.goal = None
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_step = expand_step
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = None
        self.max_err = max_err

    def get_tree(self) :
        """改进反馈控制,根据当前位姿和目标位姿计算(v,w)
        
        Args:

        Returns:
            tree (_type_): 快速随机生成树节点

        """
        return self.node_list
    
    def route_plan(self, start, goal):
        """路径规划
        
        Args:
            start (_type_): 起始点
            goal (_type_): 终止点

        Returns:
            path (_type_): 路径

        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.node_list = [self.start]
        path = None
        last_path_length = float('inf')

        for i in range(self.max_iter):
            # 1. 在环境中随机采样点
            rnd = self.sample()

            # 2. 找到结点树中距离采样点最近的结点
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            nearest_node = self.node_list[n_ind]

            # 3. 在采样点的方向生长一个步长，得到下一个树的结点。
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_node = self.get_new_node(theta, n_ind, nearest_node)

            # 4. 检测碰撞，检测到新生成的结点的路径是否会与障碍物碰撞
            no_collision = self.check_segment_collision(new_node.x, new_node.y, nearest_node.x, nearest_node.y)
            if no_collision:
                # 改进一：当生成新的结点时，判断在一定范围内的结点是否有比当前父结点路径更有的结点
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)

                self.node_list.append(new_node)
                # 改进二：范围内的结点重新相连
                self.rewire(new_node, near_inds)

                # 判断新结点是否临近目标点
                if self.is_near_goal(new_node):
                    if self.check_segment_collision(new_node.x, new_node.y,
                                                    self.goal.x, self.goal.y):
                        last_index = len(self.node_list) - 1
                        temp_path = self.get_final_course(last_index)        # 回溯路径
                        temp_path_length = self.get_path_len(temp_path)           # 计算路径的长度
                        if last_path_length > temp_path_length:
                            path = temp_path
                            last_path_length = temp_path_length
                            print("当前的路径长度为：{}".format(last_path_length))
                        #print(path)
                        return path
        print("寻找路径失败")

    def sample(self):
        """ 在环境中采样点的函数，以一定的概率采样目标点 """
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_rand[0], self.max_rand[0]),
                   random.uniform(self.min_rand[1], self.max_rand[1])]
        else:
            rnd = [self.goal.x, self.goal.y]
        return rnd

    @staticmethod
    def get_nearest_list_index(nodes, rnd):
        """ 计算树中距离采样点距离最近的结点 """
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2
                  for node in nodes]
        min_index = d_list.index(min(d_list))
        return min_index

    def get_new_node(self, theta, n_ind, nearest_node):
        """ 计算新结点 """
        new_node = copy.deepcopy(nearest_node)

        new_node.x += self.expand_step * math.cos(theta)
        new_node.y += self.expand_step * math.sin(theta)

        new_node.cost += self.expand_step
        new_node.parent = n_ind

        return new_node

    def check_segment_collision(self, x1, y1, x2, y2):
        """ 检测碰撞 """
        for item in self.obstacle_list: #这里弄错了，不用for，检查一次就行了，之后改
            if len(item) <5:
                (ox, oy, radius) = item
                dd = self.distance_squared_point_to_segment(
                    np.array([x1, y1]),
                    np.array([x2, y2]),
                    np.array([ox, oy])
                )
                if dd <= radius ** 2:
                    return False
            else:
                estilimate_time = 5
                (ox, oy, radius,vx,vy) = item
                for i in range(estilimate_time):
                    dd = self.distance_squared_point_to_segment(
                    np.array([x1, y1]),
                    np.array([x2, y2]),
                    np.array([ox, oy])
                )
                if dd <= radius ** 2:
                    return False
        return True

    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        """ 计算线段 vw 和 点 p 之间的最短距离"""
        if np.array_equal(v, w):    # 点 v 和 点 w 重合的情况
            return (p - v).dot(p - v)

        l2 = (w - v).dot(w - v)     # 线段 vw 长度的平方
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)
        return (p - projection).dot(p - projection)

    def find_near_nodes(self, new_node):
        """ 获取新结点一定范围内的树中的结点 """
        n_node = len(self.node_list)
        r = 25*self.expand_step * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2
                  for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds

    def choose_parent(self, new_node, near_inds):
        """ 选择处在区域中的当前树的结点中 cost 最小的结点作为其新的父结点"""
        if len(near_inds) == 0:
            return new_node

        d_list = []
        for i in near_inds:
            dx = new_node.x - self.node_list[i].x
            dy = new_node.y - self.node_list[i].y
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            if self.check_collision(self.node_list[i], theta, d):
                d_list.append(self.node_list[i].cost + d)
            else:
                d_list.append(float('Inf'))

        min_cost = min(d_list)
        min_ind = near_inds[d_list.index(min_cost)]

        if min_cost == float('Inf'):
            print("min cost is inf")
            return new_node

        new_node.cost = min_cost
        new_node.parent = min_ind

        return new_node

    def check_collision(self, near_node, theta, d):
        tmp_node = copy.deepcopy(near_node)
        end_x = tmp_node.x + math.cos(theta) * d
        end_y = tmp_node.y + math.sin(theta) * d
        return self.check_segment_collision(tmp_node.x, tmp_node.y, end_x, end_y)

    def rewire(self, new_node, near_inds):
        """ 范围内的结点重连"""
        n_node = len(self.node_list)
        for i in near_inds:
            near_node = self.node_list[i]
            d = math.sqrt((near_node.x - new_node.x) ** 2 + (near_node.y - new_node.y) ** 2)
            s_cost = new_node.cost + d

            if near_node.cost > s_cost:
                theta = math.atan2(new_node.y - near_node.y, new_node.x - near_node.x)
                if self.check_collision(near_node, theta, d):
                    near_node.parent = n_node - 1
                    near_node.cost = s_cost

    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.max_err :
            return True
        return False

    @staticmethod
    def line_cost(node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def get_final_course(self, last_index):
        """ 回溯路径 """
        #path = [[self.goal.x, self.goal.y]]
        path = []
        while self.node_list[last_index].parent is not None:
            node = self.node_list[last_index]
            path.append([node.x, node.y])
            last_index = node.parent
        path.append([self.start.x, self.start.y])
        return path

    @staticmethod
    def get_path_len(path):
        """ 计算路径的长度 """
        path_length = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            path_length += math.sqrt((node1_x - node2_x) ** 2 + (node1_y - node2_y) ** 2)
        return path_length


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():

    print('Start RRT planning!')
    show_animation = True
    start = [0, 0]
    goal = [15, 12]
    # 障碍物 (x, y, radius)
    obstacle_list = [
        (3, 3, 1.5),
        (12, 2, 3),
        (3, 9, 2),
        (9, 11, 2)
    ]

    rrt = RRT(rand_area=[-2, 18], obstacle_list=obstacle_list, max_iter=200)
    path = rrt.route_plan(start=[0, 0], goal=[15, 12])
    print('Done!')

if __name__ == '__main__':
    main()