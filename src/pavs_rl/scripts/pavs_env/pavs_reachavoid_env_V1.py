#!/usr/bin/env python

import gym
import math
from gym.envs.registration import register
import rospkg
from geometry_msgs.msg import Twist
from frobs_rl.common import ros_gazebo
from frobs_rl.common import ros_controllers
from frobs_rl.common import ros_node
from frobs_rl.common import ros_spawn
from frobs_rl.envs.robot_BasicEnv import RobotBasicEnv
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from gym import spaces




class PavsReachAvoidEnv(RobotBasicEnv):

    def __init__(self):

        rospy.loginfo(f"\033[94m PAVS RL Environment initializing.\033[0m")

        launch_gazebo = True
        gazebo_init_paused = False
        gazebo_use_gui = True
        gazebo_recording = False
        gazebo_freq = 100
        '''
        如果使用自定义世界启动 Gazebo，请设置相应的环境变量。
        '''
        pkg_path = '/home/jianghan/gazebo_rl/gazebo_rl/src/car-like-robot-gazebo/robot_gazebo'
        world_path = pkg_path +'/worlds/ROS-Academy.world'
        world_pkg = 'robot_gazebo'
        world_filename = 'ROS-Academy.world'
        
        gazebo_max_freq = None
        gazebo_timestep = None
        spawn_robot = True
        model_name_in_gazebo = "pav_s00"
        namespace = "/pav_s00"
        pkg_name = 'robot_gazebo'
        urdf_file = 'pav_s00.xacro'
        urdf_folder ='/urdf/pav_s00'
        controller_file = 'controller_manager/pav_s00_motor_ctrl_config.yaml'
        # controller_file = 'pav_s00_motor_ctrl_config.yaml'
        controller_list = ['left_front_wheel_velocity_controller',
                           'right_front_wheel_velocity_controller',
                           'right_rear_wheel_velocity_controller',
                           'left_rear_wheel_velocity_controller',
                           'left_steering_hinge_position_controller',
                           'right_steering_hinge_position_controller',
                           'joint_state_controller']
        urdf_xacro_args = None
        model_pos_x = 0.0
        model_pos_y = 0.0
        model_pos_z = 0.0
        model_ori_x = 0.0
        model_ori_y = 0.0
        model_ori_z = 0.0
        model_ori_w = 0.0
        reset_controllers = False
        reset_mode = 1
        step_mode = 1
        num_gazebo_steps = 1

        super(PavsReachAvoidEnv, self).__init__(launch_gazebo=launch_gazebo, gazebo_init_paused=gazebo_init_paused, 
                                                gazebo_use_gui=gazebo_use_gui, gazebo_recording=gazebo_recording,
                                                gazebo_freq=gazebo_freq, world_path=world_path, world_pkg=world_pkg,
                                                world_filename=world_filename, gazebo_max_freq=gazebo_max_freq,
                                                gazebo_timestep=gazebo_timestep, spawn_robot=spawn_robot, 
                                                model_name_in_gazebo=model_name_in_gazebo,namespace=namespace, pkg_name=pkg_name,
                                                urdf_file=urdf_file, urdf_folder=urdf_folder,  controller_file=controller_file, 
                                                controller_list=controller_list, urdf_xacro_args=urdf_xacro_args,
                                                model_pos_x=model_pos_x, model_pos_y=model_pos_y, model_pos_z=model_pos_z,
                                                model_ori_x=model_ori_x, model_ori_y=model_ori_y, model_ori_z=model_ori_z, model_ori_w=model_ori_w,
                                                reset_controllers=reset_controllers, reset_mode=reset_mode, step_mode=step_mode,
                                                num_gazebo_steps=num_gazebo_steps)
        
        '''
        按需要定义订阅和发布话题
        '''
        # 初始化ROS节点和速度发布器
        rospy.init_node('pavs_env_node', anonymous=True)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        
        '''
        定义初始状态
        '''
        #定义初始的位置
        self.start_position = [model_pos_x, model_pos_y, model_pos_z] 
        #定义目标位置
        self.target_position = [model_ori_x, model_ori_y, model_ori_z,model_ori_w]
        #定义小车的当前位置，即初始位置为A点
        self.current_position = self.start_position.copy() 
        #定义小车的下一个位置
        self.next_position = []
        '''
        定义状态空间和动作空间
        '''
        # 定义观测空间
        self.observation_space = spaces.Box(low=np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32),
                                            high=np.array([10.0, 10.0, 10.0, 10.0], dtype=np.float32),
                                            dtype=np.float32)  # 示例范围设定为 [0, 10]，具体根据需要调整
        # 定义动作空间
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)


        

    
        
        # 初始化小车状态
        self.car_position = np.array([0.0, 0.0], dtype=np.float32)  # 初始位置
        self.car_velocity = np.array([0.0, 0.0], dtype=np.float32)  # 初始速度

        # 初始化机器人位置和目标位置
        self.robot_position = np.array([1, 1], dtype=np.float32)  # 机器人初始位置
        self.target_position = np.array([9, 9], dtype=np.float32)  # 目标位置

        rospy.loginfo("Finished Init of PAVS RL Environment")


    def reset(self):
        # 重置环境到初始状态
        rospy.loginfo("Reset PAVS RL Environment Resource!")
        self._set_episode_init_params()
        return self._get_observation()
    

    def step(self, action):
        # 执行动作并返回新的状态、奖励、是否终止和附加信息
        self._send_action(action)
        observation = self._get_observation()
        reward = self._get_reward()
        done = self._check_if_done()
        return observation, reward, done, {}   
    
    def render(self, mode='human'):
        # 可视化环境，可选方法
        pass   

    def close(self):
        # 清理环境资源，可选方法
        # 停止ROS节点
        rospy.signal_shutdown("Environment closed")

    def _check_subs_and_pubs_connection(self):
        """
        Function to check if the Gazebo and ROS connections are ready
        """
        return True

    def _send_action(self, action):
        """
        将动作发送到机器人控制器或仿真环境中
        """
        linear_velocity = action[0]   #获取线速度
        angular_velocity = action[1]  #获取角速度
         # 创建一个Twist消息对象
        twist_cmd = Twist()
         # 设置线速度和角速度
        twist_cmd.linear.x = linear_velocity  # linear_velocity 是线速度
        twist_cmd.angular.z = angular_velocity  # angular_velocity 是角速度
         # 发布速度控制命令到/cmd_vel主题
        self.cmd_vel_publisher.publish(twist_cmd)
    def _get_observation(self):

        # 设置订阅者获取当前位置（从里程计消息）
        self.current_position = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
         # 设置订阅者获取激光扫描数据
        self.laser_scan = None
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

        
         # 等待直到获取到当前位置和激光扫描数据
        while self.current_position is None or self.laser_scan is None:
            rospy.sleep(0.1)

        # 组合观测值
        observation = {
            'current_position': self.current_position,
            'goal_position': self.ori_position,
            'laser_scan': self.laser_scan
        }
        # 返回观测值
        return observation

    def _get_reward(self):
        """
        从环境中获得奖励的功能。
        """
        self.reward = 0.0

        dist_curr = np.linalg.norm(np.array(self.current_position[:2]) - np.array(self.target_position[:2]))
        dist_next = np.linalg.norm(np.array(self.next_position[:2]) - np.array(self.target_position[:2]))

        self.reward_goal = dist_curr - dist_next  # goal reward. close to goal, reward_goal is larger, else negative.
        self.reward += self.reward_goal

        if self._obstacle_detection():
            self.reward_obs = -1  # obstacle reward, negative value.
            self.reward += self.reward_obs
            is_collision = True
        else:
            is_collision = False

        return is_collision
    
    def _set_obstacle(self, positions):

        self.positon_obstacle = []
        for pos in positions:
            self.positon_obstacle.append(pos)
    
    def _obstacle_detection(self):
       
        is_collision = False

        d_min = float('inf')
        for pos in self.positon_obstacle:
            d = np.linalg.norm(np.array(self.current_position[:2]) - np.array(pos[:2]))
            if d < d_min:
                d_min = d

        if d_min < 0.1:
            is_collision = True

        return is_collision

    def _check_if_done(self):


        done = False
        self.info = {'is_success': 0.0}

        goal_threshold = 0.1
        distance_goal = np.linalg.norm(np.array(self.current_position[:2]) - np.array(self.target_position))

        if distance_goal < goal_threshold:
            done = True
            self.info['is_success'] = 1.0
            return done

        obstacle_threshold = 0.2

        for obs in self.positon_obstacle:
            distance_obstacle = np.linalg.norm(np.array(self.current_position[:2]) - np.array(obs))
            if distance_obstacle < obstacle_threshold:
                done = True
                return done

        raise done

    def _set_episode_init_params(self):
        """
        用于在每次的episode开始时设置一些参数，如机器人的位置
        """
        self.start_position =  np.array([0.0, 0.0, 0.0])   # 重置机器人位置
        self.target_position = np.array([9.0, 0.0, 0.0])   # 目标位置


    def odom_callback(self, msg):
            position = msg.pose.pose.position
            self.current_position = [position.x, position.y, position.z]
    def laser_scan_callback(self, msg):
            self.laser_scan = np.array(msg.ranges)

if __name__ == "__main__":
    env = PavsReachAvoidEnv()

    # 测试reset()方法
    obs = env.reset()
    print("初始观测值：", obs)

    # 执行几个动作步骤
    for _ in range(10):
        action = env.action_space.sample()  # 随机选择一个动作
        obs, reward, done, _ = env.step(action)
        print(f"动作：{action}, 新的观测值：{obs}, 奖励：{reward}, 是否终止：{done}")

    env.close()

  
    # def _get_observation(self):

    #     # # 设置订阅者获取当前位置（从里程计消息）
    #     # self.current_position = None
    #     # rospy.Subscriber('/odom', Odometry, odom_callback)
    #     #  # 设置订阅者获取激光扫描数据
    #     # self.laser_scan = None
    #     # rospy.Subscriber('/scan', LaserScan, laser_scan_callback)


    #     # def odom_callback(msg):
    #     #     position = msg.pose.pose.position
    #     #     self.current_position = [position.x, position.y, position.z]

    #     # def laser_scan_callback(msg):
    #     #     self.laser_scan = np.array(msg.ranges)
        
    #     #  # 等待直到获取到当前位置和激光扫描数据
    #     # while self.pos_curr is None or self.laser_scan is None:
    #     #     rospy.sleep(0.1)
    #     # self.current_position = (1.0, 0.0, 0.0)  # 假设小车向前移动了1米
    #     # self.ori_position = (0.0, 0.0, 0.0)  # 假设初始目标位置为原点
    #     # self.laser_scan = [1.0] * 360  # 假设激光扫描数据初始化为距离1.0米

    #     # 组合观测值
    #     observation = {
    #         'current_position': self.pos_curr,
    #         'goal_position': self.ori_position,
    #         'laser_scan': self.laser_scan
    #     }

    #     # 返回观测值
    #     return observation

    # def _get_reward(self):
        
    #     self.reward = 0.0
    #     # 计算当前位置与目标位置之间的欧氏距离
    #     dist_curr = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(self.pos_goal[:2]))
    #     # 计算下一个位置与目标位置之间的欧氏距离。
    #     dist_next = np.linalg.norm(np.array(self.pos_next[:2]) - np.array(self.pos_goal[:2]))
    #     # 设置目标奖励
    #     self.reward_goal = dist_curr - dist_next  # goal reward. close to goal, reward_goal is larger, else negative.
    #     self.reward += self.reward_goal
    #     # 检测障碍物的函数
    #     if self._obstacle_detection():
    #         # 给一个负的奖励
    #         self.reward_obs = -1  # obstacle reward, negative value.
    #         self.reward += self.reward_obs
    #         is_collision = True
    #     else:
    #         is_collision = False

    #     return is_collision

    # def _obstacle_detection(self):
    
    #     is_collision = False

    #     d_min = float('inf')
    #     for pos in self.pos_obs:
    #         # 计算当前机器人位置 self.pos_curr 与每个障碍物位置 pos 之间的欧氏距离
    #         d = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(pos[:2]))
    #         # 记录最近的障碍物距离。
    #         if d < d_min:
    #             d_min = d

    #     if d_min < 0.1:
    #         is_collision = True

    #     return is_collision

    # def _check_if_done(self):
    #     """
    #     Function to check if the episode is done.

    #     If the episode has a success condition then set done as:
    #         self.info['is_success'] = 1.0

    #     params:
    #     ------
    #         - goal_threshold： The threshold of goal, and now let this hyper-parameter is 0.1.
    #         - obstacle_threshold: The threshold of obstacle, and now let this hyper-parameter is 0.1.
    #     returns:
    #     --------
    #         - if_done(bool): True means done, False means not done.

    #     """
    #     done = False
    #     self.info = {'is_success': 0.0}

    #     goal_threshold = 0.1
    #     distance_goal = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(self.pos_goal))

    #     if distance_goal < goal_threshold:
    #         done = True
    #         self.info['is_success'] = 1.0
    #         return done

    #     obstacle_threshold = 0.2

    #     for obs in self.pos_obs:
    #         distance_obstacle = np.linalg.norm(np.array(self.pos_curr[:2]) - np.array(obs))
    #         if distance_obstacle < obstacle_threshold:
    #             done = True
    #             return done

    #     raise done

    # def _set_obstacle(self, positions):
    #     """
    #     Function to set obstacles position.
    #     """
    #     self.pos_obs = []
    #     for pos in positions:
    #         self.pos_obs.append(pos)

    # def _set_goal(self, pos):
    #     """
    #     Function to set goal position.
    #     """
    #     self.pos_goal = pos

    # def _reset_gazebo(self):
    #     """
    #     Function to reset the gazebo simulation.
    #     """

    #     # If resetting world (Does not reset time)
    #     if self.reset_mode == 1:
    #         ros_gazebo.gazebo_reset_world()

    #     # If resetting simulation (Resets time)
    #     elif self.reset_mode == 2:
    #         ros_gazebo.gazebo_reset_sim()

    #     if self.reset_controllers:
    #         ros_gazebo.gazebo_unpause_physics()
    #         ros_controllers.reset_controllers_srv(self.controllers_list, ns=self.namespace)
    #         ros_gazebo.gazebo_pause_physics()

    #     ros_gazebo.gazebo_unpause_physics()
    #     # self._check_subs_and_pubs_connection()
    #     # self._set_episode_init_params()
    #     ros_gazebo.gazebo_pause_physics()
