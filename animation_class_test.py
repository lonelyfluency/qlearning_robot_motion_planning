import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import pickle
import random

from dynamic_obstacles import DynaObs
from qlearning_motion_planning import Map
from qlearning_motion_planning import Robot

# class Robot:
#     def __init__(self, x, y):
#         self.pos = (x, y)


class Animator:
    def __init__(self, fig, ax, g_map, robot, dyna_obs=None):
        self.ax = ax
        self.g_map = g_map
        self.robot = robot
        self.target = g_map.TerminalPoint
        self.static_obs = np.array(g_map.StaticObstacle)
        self.dyna_obs = dyna_obs
        self.ani = animation.FuncAnimation(fig, func=self.animate, init_func=self.init, repeat=False, interval=50)
        self._robot_xs = [robot.pos[0]]
        self._robot_ys = [robot.pos[1]]
        self._robot_trajectory, = self.ax.plot([], [], 'r-', linewidth=1.5, label='trajectory')
        self._robot_pos = self.ax.scatter(robot.pos[0], robot.pos[1], marker=(4, 0, 0), s=50, c='b', label='car')
        self._target_pos = self.ax.scatter(self.target[0], self.target[1], marker='*', s=100, label='target')
        self._static_obs_pos = self.ax.scatter(self.static_obs[:, 0], self.static_obs[:, 1],
                                               marker='o', s=30, c='g', label='static ob')
        self._dyna_obs_pos = self.ax.scatter(self.dyna_obs.ob_states[:, 0], self.dyna_obs.ob_states[:, 1],
                                             marker='o', s=30, c='m', label='dyna ob')
        self.ax.legend()

    def init(self):
        self.ax.set_xlim(-5, 38)
        self.ax.set_ylim(-5, 35)
        return self._robot_trajectory, self._robot_pos

    def animate(self, i):
        # self.robot.x += 0.5
        # self.robot.y += 0.5
        # self.robot.pos = (self.robot.pos[0] + 0.1, self.robot.pos[1] + 0.1)
        self.robot.run(self.g_map)
        self._robot_xs.append(self.robot.pos[0])
        self._robot_ys.append(self.robot.pos[1])
        self._robot_trajectory.set_data(self._robot_xs, self._robot_ys)

        # self._robot_pos.set_offsets([self.robot.x, self.robot.y])
        self._robot_pos.remove()
        self._robot_pos = self.ax.scatter(self.robot.pos[0], self.robot.pos[1], marker=(4, 0, 0), s=100, label='car')

        self.dyna_obs.update()
        self._dyna_obs_pos.remove()
        self._dyna_obs_pos = self.ax.scatter(self.dyna_obs.ob_states[:, 0], self.dyna_obs.ob_states[:, 1],
                                             marker='o', s=30, c='m', label='dyna ob')

        # stop when robot reaches target
        if (self.robot.pos[0] - self.target[0])**2 + (self.robot.pos[1] - self.target[1])**2 < 0.2:
            self.ani.event_source.stop()

        return self._robot_trajectory, self._robot_pos


if __name__ == '__main__':
    dynamic_obs = DynaObs(5)
    dynamic_obs.initial(x_lim=[0, 30], y_lim=[0, 30], velocity_lim=[2, 0.8], method='random')

    sta_obst = [(3, 3), (4, 4), (8, 8)]
    world_map = Map(start_point=(0, 0), terminal=(30, 30), sta_obstacle=sta_obst)
    robot1 = Robot()
    robot1.init_robot(world_map)
    # robot1.init_Q()
    robot1.read_q_table()

    # robot1 = Robot(0, 0)

    fig, ax = plt.subplots()
    animator = Animator(fig, ax, world_map, robot1, dynamic_obs)
    plt.show()
