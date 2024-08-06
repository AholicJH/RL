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
        world_path = pkg_path +'/worlds/goal_world.world'
        world_pkg = 'robot_gazebo'
        world_filename = 'goal_world.world'
        
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
        # rospy.init_node('pavs_env_node', anonymous=True)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
         # 设置发布频率，例如10Hz
        rate = rospy.Rate(10)
        self.current_position = None
        self.pos_curr = np.zeros(3)  # Initialize current position
        self.pos_prev = np.zeros(3)  # Initialize previous position
        self.laser_scan = None
        #设置订阅者获取当前位置（从里程计消息）
        
        rospy.Subscriber('/pav_s03/odom', Odometry, self.odom_callback)
        
        #设置订阅者获取激光扫描数据
        
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.positon_obstacle =  [
            [10.120900, -4.616070, 0.315032],
            [8.476070, -8.359580, 0.153570],
            [7.260040, 4.277960, 0.239029],
            [4.580360, 0.944566, 0.383762],
            [0.790650, 7.352590, 0.358593],
            [1.296639, -2.888562, 0.392052],
            [2.000000, -6.000000, 0.014020],
            [-5.611890, 4.459350, 0.378775],
            [-6.271170, 1.438670, 0.349874],
            [-6.573570, -4.581410, 0.263004]
        ]  # 初始化障碍物位置
        
        '''
        定义状态空间和动作空间
        '''
        # 定义观测空间
        # self.observation_space = spaces.Box(low=np.array([-10.0, -10.0, -10.0, -10.0, -10.0, -10.0,-10.0, -10.0, -10.0], dtype=np.float32),
        #                                     high=np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0], dtype=np.float32),
        #                                     dtype=np.float32)  # 示例范围设定为 [0, 10]，具体根据需要调整
         # 定义状态空间
        low = np.full((66,), -10.0, dtype=np.float32)
        high = np.full((66,), 10.0, dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        # 定义动作空间
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)


        # 初始化小车状态
        # self.car_position = np.array([0.0, 0.0], dtype=np.float32)  # 初始位置
        self.car_velocity = np.array([0.0, 0.0], dtype=np.float32)  # 初始速度

        # 初始化机器人位置和目标位置   target_position
        self.robot_position = np.array([0, 0, 0], dtype=np.float32)  # 机器人初始位置
        self.target_position = np.array([2.000000, -6.000000, 0.014020], dtype=np.float32)  # 目标位置
        #定义小车的下一个位置
        self.next_position =  np.array([4, 4, 0], dtype=np.float32)
        # 初始化步数计数器
        self.current_step = 0  

        rospy.loginfo("Finished Init of PAVS RL Environment")
        # 用于在 ROS 和 Gazebo 中暂停物理仿真
        # ros_gazebo.gazebo_pause_physics()

    def reset(self):
        
        # self.laser_scan = None  # 或者根据需要重新生成雷达数据
        self._reset_gazebo()
        obs = self._get_observation()
        # ros_gazebo.gazebo_reset_world()
        rospy.loginfo("Reset PAVS RL Environment Resource!")
        return obs
        # return True
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
            # self._check_subs_and_pubs_connection()
            self._set_episode_init_params()
            ros_gazebo.gazebo_pause_physics()

    def step(self, action):
        # 执行动作并返回新的状态、奖励、是否终止和附加信息
         # 处理雷达数据
        done = False
        
        # laser_data = self._process_laser_scan()
        # 根据雷达数据修改动作
        # action = self.decide_action(laser_data, action)        
        self._send_action(action)
        self._step_simulation()
        obs = self._get_observation()
        reward = self._get_reward()
        # done = self._is_done()
        self.current_step += 1  # 增加当前时间步

        done = self._check_if_done()

        # 添加附加信息
        info = {}  # 初始化 info 为空字典
        info['time_step'] = self.current_step  # 设置当前时间步数
       
        self.rate.sleep()  # 按照设定的频率暂停
         # 检查是否达到终止条件
        
        

        return obs, reward, done, info  
        # return info  
    
    
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
    
    def decide_action(self, laser_data, original_action):
        min_left, min_front, min_right = laser_data
        
        if min_front < 0.5:
            if min_left > min_right:
                return np.array([-0.3, 0.3])  # 左转
            else:
                return np.array([0.3, -0.3])  # 右转
        return original_action  # 保持原来的动作
    

    def _send_action(self, action):

        """
        将动作发送到机器人控制器或仿真环境中
        """
        linear_velocity = action[0]   #获取线速度
        angular_velocity = action[1]  #获取角速度
        rospy.loginfo(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")
         # 创建一个Twist消息对象
        twist_cmd = Twist()
         # 设置线速度和角速度
        twist_cmd.linear.x = linear_velocity  # linear_velocity 是线速度
        twist_cmd.angular.z = angular_velocity  # angular_velocity 是角速度
         # 发布速度控制命令到/cmd_vel主题
       
        self.cmd_vel_publisher.publish(twist_cmd)
        # 控制发布频率
        # self.rate.sleep()
    
    def _process_laser_scan(self):
        '''
        通过将雷达数据分为三个部分：左侧、中间和右侧，并获取每个部分的最小距离,从而判断障碍物的分布情况
        '''
        while self.laser_scan is None:
                rospy.sleep(0.1)

        third_len = len(self.laser_scan) // 3
        left_sector = self.laser_scan[:third_len]
        front_sector = self.laser_scan[third_len:2*third_len]
        right_sector = self.laser_scan[2*third_len:]
        min_left = np.min(left_sector)
        min_front = np.min(front_sector)
        min_right = np.min(right_sector)

        return np.array([min_left, min_front, min_right])

    def _get_observation(self):
   

        
         
        # 将处理后的
        # self.laser_scan = self._process_laser_scan()
        # 组合观测值
        observation = {
            'current_position': self.current_position,
            'goal_position': self.target_position,
            'laser_scan': self.laser_scan
        }
        
        # observation = np.concatenate((self.current_position, self.target_position,self.laser_scan))
        # 返回观测值
        return observation

    def _get_reward(self):

        """
        从环境中获得奖励的功能。
        """
         # 控制发布频率
        # self.rate.sleep()
        reward = 0.0
        dist_prev = np.linalg.norm(np.array(self.pos_prev) - np.array(self.target_position))
        dist_curr = np.linalg.norm(np.array(self.pos_curr) - np.array(self.target_position))
        rospy.logwarn(f"Distance from previous position to target: {dist_prev}")
        rospy.logwarn(f"Distance from current position to target: {dist_curr}")
        # reward_goal = 10 * (dist_curr - dist_prev)   # goal reward. close to goal, reward_goal is larger, else negative.
        # reward += reward_goal

        if dist_curr < dist_prev:
            reward += 10  # 如果机器人接近目标，则给予积极奖励
        else:
            reward -= 10  # 如果机器人远离目标，则给予负奖励

        rospy.loginfo("Reward Of Pavs03 ：  " + str(reward))
        # 碰撞惩罚
        # if self._check_collision():
        #   rospy.logwarn("发送碰撞")
        #   reward -= 10  # 如果发生碰撞，则给予大的负奖励
        #   rospy.loginfo("Reward Of Pavs03 ：  " + str(reward))  # 将 reward 转换为字符串


        # 终止条件额外奖励
        if dist_curr < 0.1:  # 假设0.1米为成功到达目标的阈值
            reward += 50  # 给予一个额外的正奖励
            rospy.loginfo("Goal reached, additional reward: 50")
        
        
        return reward
    

        # if self._obstacle_detection():
        #     self.reward_obs = -1  # obstacle reward, negative value.
        #     self.reward += self.reward_obs
        #     is_collision = True
        # else:
        #     is_collision = False

        # return is_collision


        # reward = 0
        # distance_to_target = np.linalg.norm(self.current_position - self.target_position)
        # reward = -distance_to_target
        # if distance_to_target < 1.0:
        #     # rospy.logwarn("给与奖励：" + str(reward))
        #     reward += 10
        #     rospy.logwarn("给与奖励：" + str(reward))
        # rospy.loginfo("Reward Of Pavs03 ：  " + str(reward))  # 将 reward 转换为字符串
       
        # # 检测与障碍物的距离
        # # if self._obstacle_detection():
        # if self._check_collision():
        #    rospy.logwarn("发送碰撞")
        #    reward -= 10  # 如果发生碰撞，则给予大的负奖励
        #    rospy.loginfo("Reward Of Pavs03 ：  " + str(reward))  # 将 reward 转换为字符串
        #    # done

        # return reward
    
    def _set_obstacle(self, positions):

        self.positon_obstacle = []
        for pos in positions:
            self.positon_obstacle.append(pos)

    def _check_collision(self):  
        '''
        检查雷达数据，判断是否与障碍物发生碰撞
        '''
        collision_threshold = 0.2  # 碰撞距离阈值
       
        if np.any(self.laser_scan < collision_threshold):
            return True
        return False
  

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
        '''
        检查是否完成
        '''
        done = False
        self.info = {'is_success': 0.0}

        goal_threshold = 0.15
        distance_goal = np.linalg.norm(np.array(self.current_position) - np.array(self.target_position))

        if distance_goal < goal_threshold:
            done = True
            self.info['is_success'] = 1.0
            ros_gazebo.gazebo_pause_physics()
            return done

        obstacle_threshold = 0.2

        for obs in self.positon_obstacle:
            distance_obstacle = np.linalg.norm(np.array(self.current_position) - np.array(obs))
            if distance_obstacle < obstacle_threshold:
                done = True
                return done
            
        if self.current_step >= 200:
            done = True
            ros_gazebo.gazebo_reset_world()
            rospy.loginfo("Episode done: Reached 200 steps.")

        return done
    
    def odom_callback(self, msg):
        '''
        获取里程计信息
        '''
        self.current_position = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])
        
        self.pos_prev = self.pos_curr.copy()

        self.pos_curr = self.current_position.copy()
        # rospy.loginfo(f"pos_prev to target: {self.pos_prev}")
        # rospy.loginfo(f"pos_curr to target: {self.pos_curr}")

    def laser_scan_callback(self, msg):
        '''
        获取雷达信息
        '''
        # self.laser_scan = msg
        # 处理雷达数据，获取有效距离数据
        ranges = np.array(msg.ranges)
        ranges[ranges == float('inf')] = msg.range_max  # 将无穷远值替换为最大测量距离
        self.laser_scan = ranges
        # rospy.loginfo(f" pos lidar data")

    def _is_done(self):
        distance_to_target = np.linalg.norm(self.current_position - self.target_position)
        if distance_to_target < 0.1:
            return True
        return False

    def _step_simulation(self):
        '''
        在模拟仿真过程中会每隔0.1秒执行一次暂停
        '''
        rospy.sleep(0.1)

    def laser_callback(self, data):
        self.laser_scan = data

    def _set_episode_init_params(self):
        # 重置环境到初始状态
        self.current_position = self.robot_position.copy()
        self.target_position = np.array([2.000000, -6.000000, 0.014020], dtype=np.float32)
        self.car_velocity = np.array([0.0, 0.0], dtype=np.float32)

if __name__ == "__main__":
    env = PavsReachAvoidEnv()

    # 测试reset()方法
    obs = env.reset()
    print("初始观测值：", obs)

    # 执行几个动作步骤
    for _ in range(1000):
        action = env.action_space.sample()  # 随机选择一个动作
        obs, reward, done, _ = env.step(action)
        print(f"动作：{action}, 新的观测值：{obs}, 奖励：{reward}, 是否终止：{done}")
        if done:
            break

    env.close()

