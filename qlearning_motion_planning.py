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

        self.FA = (self.heading_dir-math.pi/2, self.heading_dir+math.pi/2)

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

    def __get_state_R(self,g_map):
        nearest_obs_dis = self.__get_nearest_obs_dis(g_map)
        if nearest_obs_dis <= self.D_max / 3:
            self.R = 0
        elif nearest_obs_dis <= self.D_max * 2 / 3:
            self.R = 1
        elif nearest_obs_dis <= self.D_max:
            self.R = 2
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
        Vnl = self.Vlmin_dw + math.floor((n-1)/4)*self.res_l
        Vna = self.Vamin_dw + ((n-1)%10)*self.res_a
        self.Va__ = Vnl
        self.Vl__ = Vna

    def __virtual_trajectory(self):
        px = self.pos[0]
        py = self.pos[1]
        heading_theta = self.heading_dir
        for i in range(self.interval_time // self.dt):
            px += self.Vl__*math.cos(heading_theta)*self.dt
            py += self.Vl__*math.sin(heading_theta)*self.dt
            heading_theta += self.Va__*self.dt
        self.pos = (px,py)
        self.heading_dir = heading_theta

    def reward_function(self):
        Dv = self.__get_nearest_obs_dis




if __name__ == "__main__":
    sta_obst = [(5,5)]
    world_map = Map(start_point=(1,2),terminal=(3,33),sta_obstacle=sta_obst)
    robot1 = Robot()
    robot1.init_robot(world_map)

