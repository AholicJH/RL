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
        world_path = pkg_path +'/worlds/ISCAS_building.world'
        world_pkg = 'robot_gazebo'
        world_filename = 'ISCAS_building.world'
        
        gazebo_max_freq = None
        gazebo_timestep = None
        spawn_robot = True
        model_name_in_gazebo = "pav_s01"
        namespace = "/pav_s01"
        pkg_name = 'robot_gazebo'
        urdf_file = 'pav_s01.xacro'
        urdf_folder ='/urdf/pav_s01'
        controller_file = 'controller_manager/pav_s01_motor_ctrl_config.yaml'
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
        # rospy.init_node('pavs_env_node', anonymous=True)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 初始化激光扫描订阅者
        self.laser_scan = None
        
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
        # 初始化步数计数器
        self.current_step = 0  

        rospy.loginfo("Finished Init of PAVS RL Environment")


    def reset(self):
        # 重置环境到初始状态
        
        self.current_position = self.start_position.copy()
        self.car_velocity = np.array([0.0, 0.0], dtype=np.float32)
        obs = self._get_observation()
        rospy.loginfo("Reset PAVS RL Environment Resource!")
        return obs
    

    def step(self, action):
        # 执行动作并返回新的状态、奖励、是否终止和附加信息
        self._send_action(action)
        self._step_simulation()
        obs = self._get_observation()
        reward = self._get_reward()
        done = self._is_done()
    
        # 添加附加信息
        info = {}  # 初始化 info 为空字典
        info['time_step'] = self.current_step  # 设置当前时间步数
        self.current_step += 1  # 增加当前时间步

        info['robot_position'] = self.current_position

        return obs, reward, done, info  
    
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

        if self.laser_scan is None:
                laser_data = np.zeros(360, dtype=np.float32)  # 使用默认值代替 None
        else:
                laser_data = np.array(self.laser_scan.ranges, dtype=np.float32)
                laser_data = np.clip(laser_data, 0, 10)  # 截断到合理范围
                
        obs = np.concatenate([laser_data, self.car_position, self.car_velocity], axis=0)
            
        return obs

    def _get_reward(self):
        
        distance_to_target = np.linalg.norm(self.robot_position - self.target_position)
        reward = -distance_to_target
        if distance_to_target < 0.1:
            reward += 100
        return reward
    
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

    def _is_done(self):
        distance_to_target = np.linalg.norm(self.robot_position - self.target_position)
        if distance_to_target < 0.1:
            return True
        return False

    def _step_simulation(self):
        rospy.sleep(0.1)

    def laser_callback(self, data):
        self.laser_scan = data

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

