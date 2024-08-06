# import rospy
# from nav_msgs.msg import Odometry
# import numpy as np

# class YourClass:
#     def __init__(self):
#         # 初始化ROS节点
#         rospy.init_node('position_logger', anonymous=True)

#         # 初始化位置变量
#         self.current_position = None
#         self.pos_prev = np.zeros(3)  # 初始时设置为零向量
#         self.pos_curr = np.zeros(3)  # 初始时设置为零向量

#         # 订阅机器人位置信息的话题
#         rospy.Subscriber('/pav_s03/odom', Odometry, self.position_callback)

#     def position_callback(self, msg):
#         # 获取当前位置信息
#         self.current_position = np.array([msg.pose.pose.position.x,
#                                           msg.pose.pose.position.y,
#                                           msg.pose.pose.position.z])

#         # 更新前一个时刻的位置
#         self.pos_prev = self.pos_curr.copy()

#         # 更新当前时刻的位置
#         self.pos_curr = self.current_position.copy()

#         # 打印日志，验证位置信息
#         rospy.loginfo(f"Previous position: {self.pos_prev}")
#         rospy.loginfo(f"Current position: {self.pos_curr}")

# if __name__ == '__main__':
#     try:
#         your_instance = YourClass()
#         rospy.spin()  # 保持节点运行，等待订阅的消息到来
#     except rospy.ROSInterruptException:
#         pass


import gym
from gym import register
from stable_baselines3.common.vec_env import DummyVecEnv


register(
    id='PavsReachAvoidEnv-v0',
    entry_point='pavs_env.pavs_reachavoid_env:PavsReachAvoidEnv',
    max_episode_steps=100,
)




def make_env():
    return gym.make('PavsReachAvoidEnv-v0')

env = DummyVecEnv([make_env])  # 默认情况下 `num_envs` 会被sou设置为 1
print(env.num_envs)  # 打印出 `num_envs` 的值
