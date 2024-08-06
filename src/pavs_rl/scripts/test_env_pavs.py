#!/usr/bin/env python

import rospy
import gym
# import pavs_rl.scripts.pavs_env.pavs_reachavoid_env       # RL environment package
from std_msgs.msg import Float64
import time
from gym.envs.registration import register

register(
    id='PavsReachAvoidEnv-v0',
    entry_point='pavs_env.pavs_reachavoid_env:PavsReachAvoidEnv',
    max_episode_steps=1000,
)


class RLEnvTester:
    def __init__(self):
        rospy.init_node('pavs_env_tester', anonymous=True)
        self.env = gym.make('PavsReachAvoidEnv-v0')
        rospy.loginfo("测试功能！！init")

        # # 环境初始化
        # state = self.env.reset()
        # while True:
        #     # 渲染画面
        #     self.env.render()
        #     # 从动作空间随机获取一个动作
        #     action = self.env.action_space.sample()
        #     # agent与环境进行一步交互
        #     state, reward, done, info = self.env.step(action)
        #     print('state = {0}; reward = {1}'.format(state, reward))
        #     # 判断当前episode 是否完成
        #     if done:
        #         print('done')
        #         break
        #     time.sleep(0.2)
        # # 环境结束
        # self.env.close()

    def test_environment(self):
        rate = rospy.Rate(10)  # 10 Hz
        # obs = self.env.reset()
        done = False
        total_reward = 0.0

        # while not rospy.is_shutdown() and not done:
        #     action = self.env.action_space.sample()  # random action
        #     obs, reward, done, info = self.env.step(action)
        #     total_reward += reward
        #     self.reward_pub.publish(total_reward)
        #     rospy.loginfo("Step Reward: %s, Total Reward: %s", reward, total_reward)
        #     self.env.render()
        #     rate.sleep()

        # rospy.loginfo("Episode finished with total reward: %s", total_reward)
        rospy.loginfo("测试功能！！")

if __name__ == '__main__':
    try:
        tester = RLEnvTester()
        rospy.loginfo("测试功能！")
        tester.test_environment()
    except rospy.ROSInterruptException:
        pass

