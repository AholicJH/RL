#!/usr/bin/env python

import numpy as np
from frobs_rl.common import ros_controllers, ros_gazebo, ros_params
import rospy
import gym
from gym import spaces
from pavs_rl.scripts.pavs_env.pavs_reachavoid_env_goods import  PavsReachAvoidEnv  # RL environment package
from std_msgs.msg import Float64
import time
from gym.envs.registration import register

register(
    id='PavsReachAvoidEnv-v0',
    entry_point='pavs_env.pavs_reachavoid_env:PavsReachAvoidEnv',
    max_episode_steps=1000,
)

class RLEnvTester(PavsReachAvoidEnv):
    def __init__(self):

        rospy.logwarn("Starting PavsReachAvoidEnv Task Env")

        """
        Load YAML param file
        """
        ros_params.ros_load_yaml_from_pkg("pavs_rl", "task_env_pavs.yaml", ns="/pavs00")
        self.get_params()

        

        # 动作空间定义(线速度和角速度)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),   # 线速度最小值, 角速度最小值
            high=np.array([1.0, 1.0]),    # 线速度最大值, 角速度最大值
            dtype=np.float32
        )
        # 动作空间定义（例如，位置、速度、激光雷达数据）
        num_lidar_readings = 360  # 假设激光雷达有360个角度的读数
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),   # 线速度最小值, 角速度最小值
            high=np.array([1.0, 1.0]),    # 线速度最大值, 角速度最大值
            dtype=np.float32
        )

        rospy.init_node('pavs_env_tester', anonymous=True)
        self.env = gym.make('PavsReachAvoidEnv-v0')
        rospy.loginfo("测试功能！！init")

    def get_params(self):

        self.goal = rospy.get_param('/goal')  # 小车的目标位置
        self.reached_goal_reward = rospy.get_param('/reached_goal_reward')  # 达到目标的奖励
        self.step_reward = rospy.get_param('/step_reward')   # 每步的奖励
        self.mult_dist_reward = rospy.get_param('/mult_dist_reward')  # 距离目标的奖励倍数
        self.joint_limits_reward =rospy.get_param('/joint_limits_reward')   # 关节限制条件的奖励
        self.pos_curr = rospy.get_param('/')
        self.pos_next = rospy.get_param('/')

    def reset(self):
        rospy.loginfo("PavsReachAvoidEnv reset")
        # 初始化环境状态
        position = np.zeros(3)  # [x, y, theta]
        velocity = np.zeros(3)  # [vx, vy, omega]
        lidar = np.zeros(360)   # 激光雷达读数
        state = np.concatenate((position, velocity, lidar))
        return state
    def step(self, action):
        rospy.loginfo("PavsReachAvoidEnv step")
        # 根据动作更新环境状态并返回新的观测
        position = np.zeros(3)  # [x, y, theta] 根据实际情况更新
        velocity = np.zeros(3)  # [vx, vy, omega] 根据实际情况更新
        lidar = np.zeros(360)   # 激光雷达读数根据实际情况更新
        state = np.concatenate((position, velocity, lidar))
        reward = 0.0            # 根据实际情况计算
        done = False            # 根据实际情况判断
        info = {}               # 根据实际情况添加信息
        return state, reward, done, info
    

    def _get_reward(self):
        
        self.reward = 0.0

        dist_curr = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(self.pos_goal[:2]))
        dist_next = np.linalg.norm(np.array(self.pos_next[:2]) - np.array(self.pos_goal[:2]))

        self.reward_goal = dist_curr - dist_next  # goal reward. close to goal, reward_goal is larger, else negative.
        self.reward += self.reward_goal

        if self._obstacle_detection():
            self.reward_obs = -1  # obstacle reward, negative value.
            self.reward += self.reward_obs
            is_collision = True
        else:
            is_collision = False

        return is_collision
    

    def _obstacle_detection(self):
        """
        检测环境中是否发生了碰撞
        """
        is_collision = False

        d_min = float('inf')
        for pos in self.pos_obs:
            d = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(pos[:2]))
            if d < d_min:
                d_min = d

        if d_min < 0.1:
            is_collision = True

        return is_collision

    def _check_if_done(self):
        """
        Function to check if the episode is done.

        """
        done = False
        self.info = {'is_success': 0.0}

        goal_threshold = 0.1
        distance_goal = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(self.pos_goal))

        if distance_goal < goal_threshold:
            done = True
            self.info['is_success'] = 1.0
            return done

        obstacle_threshold = 0.2

        for obs in self.pos_obs:
            distance_obstacle = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(obs))
            if distance_obstacle < obstacle_threshold:
                done = True
                return done

        raise done
    # 设置障碍物
    def _set_obstacle(self, positions):
        """
        Function to set obstacles position.
        """
        self.pos_obs = []
        for pos in positions:
            self.pos_obs.append(pos)
    # 设置目标
    def _set_goal(self, pos):
        """
        Function to set goal position.
        """
        self.pos_goal = pos

    def _reset_gazebo(self):
        """
        Function to reset the gazebo simulation.
        """

        # If resetting world (Does not reset time)
        if self.reset_mode == 1:
            ros_gazebo.gazebo_reset_world()

        # If resetting simulation (Resets time)
        elif self.reset_mode == 2:
            ros_gazebo.gazebo_reset_sim()

        if self.reset_controllers:
            ros_gazebo.gazebo_unpause_physics()
            ros_controllers.reset_controllers_srv(self.controllers_list, ns=self.namespace)
            ros_gazebo.gazebo_pause_physics()

        ros_gazebo.gazebo_unpause_physics()
        self._check_subs_and_pubs_connection()
        self._set_episode_init_params()
        ros_gazebo.gazebo_pause_physics()

    def test_environment(self):
        rospy.loginfo("测试功能！！")
    
    

if __name__ == '__main__':
    try:
        tester = RLEnvTester()
        rospy.loginfo("测试功能！")
        tester.test_environment()
    except rospy.ROSInterruptException:
        pass

