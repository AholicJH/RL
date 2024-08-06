#!/usr/bin/env python

import os
import random
import gym
import math
from gym.envs.registration import register
import rospkg
from geometry_msgs.msg import Twist
from frobs_rl.common import ros_gazebo
from frobs_rl.common import ros_controllers
from frobs_rl.common import ros_node
from frobs_rl.common import ros_spawn
from gazebo_msgs.srv import SpawnModel, DeleteModel,GetModelState
from std_srvs.srv import Empty
from frobs_rl.envs.robot_BasicEnv import RobotBasicEnv
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from gym import spaces


class PavsReachAvoidEnv(RobotBasicEnv):

    def __init__(self):

        rospy.loginfo(f"\033[94m PAVS RL Environment initializing.\033[0m")
        # 用于在 ROS 和 Gazebo 中恢复物理仿真

        # ros_gazebo.gazebo_unpause_physics()

        launch_gazebo = True
        gazebo_init_paused = False
        gazebo_use_gui = True
        gazebo_recording = False
        gazebo_freq = 10
        '''
        如果使用自定义世界启动 Gazebo，请设置相应的环境变量。
        '''
        pkg_path = '/home/jianghan/gazebo_rl/gazebo_rl/src/car-like-robot-gazebo/robot_gazebo'
        world_path = pkg_path +'/worlds/empty_obstacle.world'
        world_pkg = 'robot_gazebo'
        world_filename = 'empty_obstacle.world'
        
        gazebo_max_freq = 1
        gazebo_timestep = None
        spawn_robot = True
        model_name_in_gazebo = "pav_s03"
        namespace = "/pav_s03"
        pkg_name = 'robot_gazebo'
        urdf_file = 'pav_s03.xacro'
        urdf_folder ='/urdf/pav_s03'
        controller_file = 'controller_manager/pav_s03_motor_ctrl_config.yaml'
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
        reset_controllers = True
        reset_mode = 1
        step_mode = 1
        num_gazebo_steps = 100
        

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
        
       
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_position = None
        self.pos_curr = np.zeros(3)  
        self.pos_prev = np.zeros(3)  
        self.laser_scan = np.zeros(60, dtype=np.float32) 
        
        self.current_position = None
        rospy.Subscriber('/pav_s03/odom', Odometry, self.odom_callback)
        self.lidar_subscriber = None
        self.lidar_subscriber =rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        # ROS 话题订阅
        self.cmd_vel = None
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
  
        self.reset_times = 0

        self.rate = rospy.Rate(10)  # 10 Hz
        

        '''
        定义状态空间和动作空间
        '''
        low = np.full((66,), -10.0, dtype=np.float32)
        high = np.full((66,), 10.0, dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        # 定义动作空间
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
    
        self.robot_position = np.array([0, 0, 0], dtype=np.float32)  # 机器人初始位置
        # self.target_position = np.array([2.000000, -6.000000, 0.000131], dtype=np.float32)  # 目标位置

        # 设置初始目标点
        self.initial_target_position = np.array([2.000000, -2.000000, 0.000131], dtype=np.float32)  # 目标位置
        self.target_position =  self.initial_target_position
        self.reset_target_positon = False
        # 初始化步数计数器2 -6 0.000131
        self.current_step = 0  
         # 生成并显示初始目标点


        rospy.loginfo("Finished Init of PAVS RL Environment")
        # ros_gazebo.gazebo_pause_physics()

    def cmd_vel_callback(self,msg):

        self.cmd_vel = msg
        
    def _set_episode_init_params(self):

        self.current_position = self.robot_position.copy()
        self.target_position = self.initial_target_position
        self.pos_curr = np.zeros(3)  
        self.pos_prev = np.zeros(3)  
        # 重置雷达
        if hasattr(self, 'lidar_subscriber'):
            self.lidar_subscriber.unregister()
            rospy.loginfo("Unregistered previous lidar subscriber")
            del self.lidar_subscriber
        # 确保雷达订阅器在重新订阅之前已取消订阅
        rospy.sleep(1.0)  # 增加等待时间确保订阅器取消

        # 重新初始化雷达数据
        self.laser_scan = np.zeros(60, dtype=np.float32)   # 清空旧的数据

        # 重新订阅雷达数据
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)


    def _reset_gazebo(self):
            """
            Function to reset the gazebo simulation.
            """
            # If resetting world (Does not reset time)
            if self.reset_mode == 1:
                ros_gazebo.gazebo_reset_world()

                rospy.logwarn("重置world")
                
            
            # If resetting simulation (Resets time)
            elif self.reset_mode == 2:
                ros_gazebo.gazebo_reset_sim()
            if self.reset_controllers:
                ros_gazebo.gazebo_unpause_physics()
                ros_controllers.reset_controllers_srv(self.controllers_list, ns=self.namespace)
                ros_gazebo.gazebo_pause_physics()
                rospy.logwarn("重置controllers")

            
            self._set_episode_init_params()
            rospy.logwarn("重置数据")
            # ros_gazebo.gazebo_pause_physics()
            ros_gazebo.gazebo_unpause_physics()

    def reset(self):
        
        rospy.loginfo("Reset PAVS RL Environment Resource!")
        self._reset_gazebo()
        obs = self._get_observation()
        return obs
    
    def step(self, action):

        self.info = {} 
        self._send_action(action)
        rospy.sleep(0.1)  # 等待 100 毫秒，确保 cmd_vel 更新
        obs = self._get_observation()
        
        reward = self._get_reward()

        done = self._check_if_done()

        self.current_step += 1  # 增加当前时间步

        
        return obs, reward, done, self.info  
        
    
    def render(self, mode='human'):
        # 可视化环境，可选方法
        pass   

    def close(self):
        # 清理环境资源，可选方法
        # 停止ROS节点
        rospy.signal_shutdown("Environment closed")
    

    def _send_action(self, action):

        """
        将动作发送到机器人控制器或仿真环境中
        """
        linear_velocity = action[0]   
        angular_velocity = action[1]  
        twist_cmd = Twist()
         
        twist_cmd.linear.x = linear_velocity  
        twist_cmd.angular.z = angular_velocity  
        
        self.cmd_vel_publisher.publish(twist_cmd)
    
    def _stop_robot(self):
        # 发布停止指令
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(vel_msg)
    
    def _get_observation(self):
        
        observation = np.concatenate((self.current_position, self.target_position, np.ravel(self.laser_scan)))
        # observation = np.concatenate((self.current_position, self.target_position))
        # 返回观测值
        return observation

    def _get_reward(self):

        """
        从环境中获得奖励的功能。
        """
        reward = 0.0
        dist_prev = np.linalg.norm(np.array(self.pos_prev) - np.array(self.target_position))
        dist_curr = np.linalg.norm(np.array(self.pos_curr) - np.array(self.target_position))
        reward_goal = dist_prev - dist_curr  
        reward += reward_goal
        
        # reward += 1.0 / (dist_curr + 1e-5)

        if dist_curr < 0.5:  # 假设0.2米为成功到达目标的阈值
            reward += 1  # 给予一个额外的正奖励
            rospy.logwarn("Goal reached, additional reward: 1")
            rospy.loginfo("Reward ：  " + str(reward)+" dist_prev:"+str(dist_prev)+"  dist_curr:"+str(dist_curr)+"  step"+str(self.current_step))
            # self.current_step = 0


         # 检测障碍物的函数
        if self._check_collision():
            # 给一个负的奖励
            reward_obs = -1  # obstacle reward, negative value.
            reward += reward_obs
            rospy.logwarn("reach obstacle, additional reward: -1")
            rospy.loginfo("Reward ：  " + str(reward)+" dist_prev:"+str(dist_prev)+"  dist_curr:"+str(dist_curr)+"  step"+str(self.current_step))
        
        
        # rospy.loginfo("Reward ：  " + str(reward)+" dist_prev:"+str(dist_prev)+"  dist_curr:"+str(dist_curr)+"  step"+str(self.current_step))
        return reward
    
    def _check_collision(self):  
        '''
        检查雷达数据，判断是否与障碍物发生碰撞
        '''
        collision_threshold = 0.25  # 碰撞距离阈值
        # 过滤掉超出范围的距离值（例如 NaN 或 Inf）
        #rospy.loginfo("LaserScan Range: %s", str(self.laser_scan))
        valid_distances = self.laser_scan[np.isfinite(self.laser_scan)& (self.laser_scan != 5)]
        # rospy.loginfo("valid_distances: %s", str(valid_distances))
        # 检查是否有障碍物在阈值距离内
        self.obstacles_within_threshold = valid_distances[valid_distances < collision_threshold]
        
        if len(self.obstacles_within_threshold) != 0:
            rospy.logwarn("obstacles_within_threshold: %s", str(self.obstacles_within_threshold))
            self._stop_robot()
            return True
        
        return False
         
        
    def _check_if_done(self):
        '''
        检查是否完成
        '''
        done, is_reach, is_collision = False, False, False
        self.info = {'is_success': 0.0}

        goal_threshold = 0.5
        distance_goal = np.linalg.norm(np.array(self.current_position) - np.array(self.target_position))
        if distance_goal < goal_threshold:
            is_reach = True
            self.info['is_success'] = 1.0
            rospy.logwarn("The Car is arrvided successfully.")
        
        if len(self.obstacles_within_threshold) != 0:
            is_collision = True
            self.info['is_success'] = 1.0
            rospy.logwarn("The Car is collisioned ！ .")

        done = is_reach or is_collision

        if done:
            self.reset_times += 1
            rospy.logwarn(f"early reset = {self.reset_times}.")

        #rospy.loginfo("标志位是："+str(done)+"执行steps数:"+str(self.current_step))
        return done
    
    def odom_callback(self, msg):
        self.current_position = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])
        
        self.pos_prev = self.pos_curr.copy()

        self.pos_curr = self.current_position.copy()

    def laser_scan_callback(self, msg):

        ranges = np.array(msg.ranges)
        ranges[ranges == float('inf')] = msg.range_max  # 将无穷远值替换为最大测量距离
        self.laser_scan = ranges
    
   
