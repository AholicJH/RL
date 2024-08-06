
import threading
import numpy as np
import gym
from geometry_msgs.msg import Twist
import rospy

class PavsActionWrapper(gym.Wrapper):
    """
    用于规范化动作空间
    """
    def __init__(self, env):
        # 获取行动空间
        action_space = env.action_space
        # 此包装器仅适用于连续动作空间（spaces.Box）
        assert isinstance(action_space, gym.spaces.Box), "This wrapper only works with continuous action space (spaces.Box)"
        # 读取最大/最小值
        self.low, self.high = action_space.low, action_space.high

        # 我们修改行动空间，让所有行动都位于 [-1, 1] 内
        env.action_space = gym.spaces.Box(low=-1, high=1, shape=action_space.shape, dtype=np.float32)
        if not rospy.get_node_uri():
            rospy.init_node('pavs_action_wrapper', anonymous=True)
        
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # # 初始化线速度和角速度的初始值
        # self.current_action = [0.3, 0.3]

        # # 启动发布命令的线程
        # self.thread = threading.Thread(target=self.publish_cmd_vel)
        # self.thread.start()

        # 调用父构造函数，以便稍后访问 self.env
        super(PavsActionWrapper, self).__init__(env)

    def publish_cmd_vel(self):
        """
        循环发布速度命令
        """
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = self.current_action[0]  # 线速度
            twist.angular.z = self.current_action[1]  # 角速度
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
    
    def rescale_action(self, scaled_action):
        """
        将动作从[-1, 1]调整为[低, 高]

        scaled_action: 要重新缩放的操作

        return: 重新缩放后的动作.
        
        """
        return self.low + (0.5 * (scaled_action + 1.0) * (self.high -  self.low))
    def send_action(self, action):
            
            """
            将动作发送到ROS主题以控制小车
            """
            twist = Twist()
            twist.linear.x = action[0]  # 假设 action[0] 是线速度
            twist.angular.z = action[1]  # 假设 action[1] 是角速度
            rospy.loginfo("发送命令action!")
            # 发布速度命令
            self.cmd_vel_pub.publish(twist)

    def reset(self):
        """
        重置环境
        """
        # Reset the counter
        return self.env.reset()

    def step(self, action):
        """
        action: 代理采取的行动
        return: 观测值、奖励、事件是否结束、附加信息
        """
        # Rescale action from [-1, 1] to original [low, high] interval
        rescaled_action = self.rescale_action(action)
        obs, reward, done, info = self.env.step(rescaled_action)
        return obs, reward, done, info