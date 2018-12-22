import numpy as np
import math

class Map:
    def __init__(self,**kwargs):
        self.StartPoint = kwargs['start_point']
        self.TerminalPoint = kwargs['terminal']
        self.StaticObstacle = kwargs['sta_obstacle']
        try:
            self.DynamicObstacle = kwargs['dyn_obstacle']
        except KeyError:
            self.DynamicObstacle = []
        self.obstacle = self.StaticObstacle + self.DynamicObstacle



class Robot:
    def __init__(self):
        self.global_time = 0
        self.interval_time = 0

        self.pos = (0, 0)
        self.radius = 0.2
        self.heading_dir = math.pi/2
        self.theta_target = 0
        self.distance_target = 0

        self.D_max = 6  # the critical line of the safe region and unsafe region.
        self.SR = 0
        self.Vlmax = 4
        self.Vamax = 2
        self.dv_max = 2
        self.da_max = 1
        self.dt = 0.001 
        
        self.Vlmin_dw = 0
        self.Vlmax_dw = 0
        self.Vamin_dw = 0
        self.Vamax_dw = 0
        self.res_l = 0
        self.res_a = 0

        self.R = 0
        self.A = 0
        self.E = 0
        self.VL = 0
        self.VA = 0
        self.state = (0,0,0,0,0)

        self.Vl = 0   # current state linear and angular velocity.
        self.Va = 0

        self.Vl__ = 0  # calculated action to take.
        self.Va__ = 0

        self.vf = 3
        self.wf = 0.3

        self.FA = (self.heading_dir-math.pi/2, self.heading_dir+math.pi/2)

        self.Q = {}

    def __update_robot_state(self,current_time):
        self.interval_time = current_time - self.global_time
        self.global_time = current_time
        self.FA = (self.heading_dir - math.pi / 2, self.heading_dir + math.pi / 2)


    def init_robot(self, g_map):
        self.__set_init_pos(g_map)
        self.__get_distance_target(g_map)
        self.__get_theta_target(g_map)

    def __set_init_pos(self, g_map):
        self.pos = g_map.StartPoint

    def __get_distance_target(self, g_map):
        dx = g_map.TerminalPoint[0] - self.pos[0]
        dy = g_map.TerminalPoint[1] - self.pos[1]
        self.distance_target = (dx**2 + dy**2)**(1/2)

    def __is_win_state(self,g_map):
        if self.__get_distance_target(g_map) < self.radius:
            return True
        return False

    def __get_theta_target(self, g_map):
        sin_tar = (g_map.TerminalPoint[1] - self.pos[1]) / self.distance_target
        self.theta_target = math.asin(sin_tar)

    def __dist_2_robot(self,obst):
        dx = obst[0] - self.pos[0]
        dy = obst[1] - self.pos[1]
        return (dx**2+dy**2)**(1/2)

    def __get_nearest_obstacle(self, g_map):  # neet to be modified, should consider the obstacle in FA.
        nearest_obs = min(g_map.obstacle,key=lambda x: self.__dist_2_robot(x))
        return nearest_obs

    def __get_nearest_obs_dis(self,g_map):
        nearest_obs = self.__get_nearest_obstacle(g_map)
        return self.__dist_2_robot(nearest_obs)

    def __is_fail_state(self,g_map):
        if self.__get_nearest_obs_dis(g_map) < self.radius:
            return True
        return False

    def __get_state_R(self,g_map):
        nearest_obs_dis = self.__get_nearest_obs_dis(g_map)
        if nearest_obs_dis <= self.D_max / 3:
            self.R = 0
            self.SR = 0
        elif nearest_obs_dis <= self.D_max * 2 / 3:
            self.R = 1
            self.SR = 0
        elif nearest_obs_dis <= self.D_max:
            self.R = 2
            self.SR = 0
        else:
            self.R = 3    # safe region
            self.SR = 1
    
    def __get_state_A(self,g_map):
        res = math.pi/10
        nearest_obs = self.__get_nearest_obstacle(g_map)
        nearest_obs_dis = self.__dist_2_robot(nearest_obs)
        sin_obs = (nearest_obs[1]-self.pos[1]) / nearest_obs_dis
        theta_obs = math.asin(sin_obs)
        alpha_obs = self.heading_dir - theta_obs

        for i in range(10):
            if self.FA[0] + i * res > (math.pi/2-alpha_obs):
                self.A = i
                break

    def __get_state_E(self,g_map):
        res = math.pi/4
        alpha_target = self.heading_dir - self.theta_target
        for i in range(8):
            if (i+1)*res >= alpha_target:
                self.E = i
                break

    def __get_state_VL(self,g_map):
        res = self.Vlmax / 4
        for i in range(4):
            if (i+1)*res >= self.Vl:
                self.VL = i
                break
    
    def __get_state_VA(self,g_map):
        res = self.Vamax / 4
        for i in range(10):
            if (i+1)*res >= self.Va:
                self.VA = i
                break

    def __get_state(self,g_map):
        self.__get_state_R(g_map)
        self.__get_state_A(g_map)
        self.__get_state_E(g_map)
        self.__get_state_VL(g_map)
        self.__get_state_VA(g_map)
        self.state = (self.R,self.A,self.E,self.VL,self.VA)

    def __dynamic_window(self):
        self.Vlmin_dw = max(self.Vl-self.dv_max*self.dt,0)
        self.Vamin_dw = max(self.Va-self.da_max*self.dt,-self.Vamax)
        self.Vlmax_dw = min(self.Vl+self.dv_max*self.dt,self.Vlmax)
        self.Vlmin_dw = min(self.Va+self.da_max*self.dt,self.Vamax)

        self.res_l = (self.Vlmax_dw-self.Vlmin_dw) / 4
        self.res_a = (self.Vamax_dw-self.Vamin_dw) / 10

    def __get_action_from_dw(self,n):
        self.__dynamic_window()
        Vnl = self.Vlmin_dw + math.floor((n-1)/4)*self.res_l
        Vna = self.Vamin_dw + ((n-1)%10)*self.res_a
        return Vnl,Vna

    def __virtual_trajectory(self,g_map):
        px = self.pos[0]
        py = self.pos[1]
        heading_theta = self.heading_dir
        for i in range(self.interval_time // self.dt):
            px += self.Vl__*math.cos(heading_theta)*self.dt
            py += self.Vl__*math.sin(heading_theta)*self.dt
            heading_theta += self.Va__*self.dt
        self.pos = (px,py)
        self.heading_dir = heading_theta
        self.Va = self.Va__
        self.Vl = self.Vl__
        self.__get_theta_target(g_map)

    def __reward_function(self,g_map):
        Dv = self.__get_nearest_obs_dis(g_map)
        alpha_v = self.heading_dir - self.theta_target
        self.__get_state(g_map)
        win = self.__is_win_state(g_map)
        fail = self.__is_fail_state(g_map)
        if win:
            return 2
        elif fail:
            return -1
        elif self.SR == 1:
            return 1
        else:
            return 0.05 * Dv + 0.2 * alpha_v + 0.1 * self.Vl__

    def init_Q(self):
        for r in range(3):
            for a in range(10):
                for e in range(8):
                    for vl in range(4):
                        for va in range(10):
                            self.Q[(r,a,e,vl,va)] = [0] * 40
    
    def __SR_judge(self,g_map,dis,theta):
        return (self.__dist_2_robot(g_map.TerminalPoint)-dis) + abs(theta-self.theta_target)*2

    def __gen_heur_list(self,n):
        vl,va = self.__get_action_from_dw(n)
        dv = abs(self.Vl - vl)
        da = abs(self.Va - va)
        return dv+da*2

    def __policy_run(self,g_map):
        while True:
            current_dis2target = self.__dist_2_robot(g_map.TerminalPoint)
            current_theta = self.heading_dir
            self.__get_state(g_map)
            current_state = self.state
            if self.SR == 1:
                self.Vl__ = self.vf
                self.Va__ = self.wf
                state_mem = (self.pos,self.heading_dir)
                self.__virtual_trajectory(g_map)
                if self.__SR_judge(g_map,self.__dist_2_robot(g_map.TerminalPoint),self.heading_dir) >= self.__SR_judge(g_map,current_dis2target,current_theta):
                    self.pos,self.heading_dir = state_mem
                    self.Va__ = -self.wf
                    self.__virtual_trajectory(g_map)
            else:
                heuristic_list = [self.__gen_heur_list(x) for x in range(40)]
                while True:
                    act_idx = min(heuristic_list)
                    state_mem = (self.pos,self.heading_dir)
                    vl_mem,va_mem = self.Vl,self.Va
                    current_obs_dis = self.__get_nearest_obs_dis(g_map)
                    self.Vl__,self.Va__ = self.__get_action_from_dw(act_idx)
                    self.__virtual_trajectory(g_map)
                    if self.__SR_judge(g_map,self.__dist_2_robot(g_map.TerminalPoint),self.heading_dir) >= self.__SR_judge(g_map,current_dis2target,current_theta) or self.__get_nearest_obs_dis(g_map)<current_obs_dis:
                        self.pos,self.heading_dir = state_mem
                        self.Vl, self.Va = vl_mem, va_mem
                        heuristic_list[act_idx] = 9999
                    else:
                        break

                self.Q[current_state] = self.__reward_function(g_map) 
                if  self.__is_fail_state(g_map):
                    break
            if self.__is_win_state(g_map):
                break

    def train(self,g_map,scenario=100):
        sce_num = scenario
        for i in range(sce_num):
            self.__policy_run(g_map)
    



if __name__ == "__main__":
    sta_obst = [(5,5)]
    world_map = Map(start_point=(1,2),terminal=(3,33),sta_obstacle=sta_obst)
    robot1 = Robot()
    robot1.init_robot(world_map)
    robot1.init_Q()
    robot1.train(world_map)


