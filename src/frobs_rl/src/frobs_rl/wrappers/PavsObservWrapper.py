import numpy as np
import gym

import rospy

class PavsObservWrapper(gym.Wrapper):
    """
     用于对观测空间进行规范化处理
    """
    def __init__(self, env):
        # 获取取观测空间
        observation_space = env.observation_space
        assert isinstance(observation_space, gym.spaces.Box), "This wrapper only works with continuous observation space (spaces.Box)"
        # 获取最大、最小值
        self.low, self.high = observation_space.low, observation_space.high

        # 我们修改行动空间，使所有行动都位于 [-1, 1] 内
        env.observation_space = gym.spaces.Box(low=-1, high=1, shape=observation_space.shape, dtype=np.float32)
        # 调用父构造函数，以便稍后访问 self.env
        super(PavsObservWrapper, self).__init__(env)
    
    def scale_observation(self, observation):
        """
        将动作从[-1, 1]调整为[低, 高]

        scaled_action: 要重新缩放的操作

        return: 重新缩放后的动作.
        """
        # rospy.loginfo(f"Debug: self.low = {self.low}, self.high = {self.high}, observation = {observation}")
        return ((observation - self.low) * (1.0/(0.5*(self.high-self.low)))) - 1.0

    def reset(self):
        """
        重置环境
        """
        # 
        observation = self.env.reset()
        scaled_obs = self.scale_observation(observation)
        return scaled_obs

    def step(self, action):
       
        observation, reward, done, info = self.env.step(action)
        # Rescale observation from [low, high] to [-1, 1] interval
        scaled_obs = self.scale_observation(observation)
        return scaled_obs, reward, done, info