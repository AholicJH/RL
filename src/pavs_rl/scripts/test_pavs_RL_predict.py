#!/usr/bin/env python

import sys
import gym
from gym import register
import numpy as np
import rospkg


from frobs_rl.common import ros_gazebo, ros_node
from frobs_rl.models.ppo import PPO
from frobs_rl.models.td3 import TD3
from frobs_rl.wrappers.NormalizeActionWrapper import NormalizeActionWrapper
from frobs_rl.wrappers.NormalizeObservWrapper import NormalizeObservWrapper
import rospy


register(
    id='PavsReachAvoidEnv-v0',
    entry_point='pavs_env.pavs_reachavoid_env:PavsReachAvoidEnv',
    max_episode_steps=100,
)



if __name__ == '__main__':
    
    # Kill all processes related to previous runs
    # ros_node.ros_kill_all_processes()
    
    rospy.logwarn("Start")
    rospy.init_node('train_Pavs03_predict')
    env = gym.make('PavsReachAvoidEnv-v0')  # 替换为你的环境名称或者自定义环境

    env = NormalizeActionWrapper(env)

    #--- Normalize observation space
    env = NormalizeObservWrapper(env)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("pavs_rl")
    # save_path = pkg_path + "/models/td3/"+"trained_model_25_07_2024_21_16_20"  x效果较好？
    # save_path = pkg_path + "/models/td3/"+"trained_model_26_07_2024_09_43_09"
    # save_path = pkg_path + "/models/td3/"+"trained_model_26_07_2024_17_34_13" 
    save_path = pkg_path + "/models/ppo/"+"trained_model_02_08_2024_20_14_51"
    
    # save_path = pkg_path + "/models/ppo/"+"trained_model_31_07_2024_16_42_27"
    # model = TD3.load_trained(save_path)
    model = PPO.load_trained(save_path)
    obs = env.reset()
    episodes = 20
    epi_count = 0


    while epi_count < episodes:
         
    
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, dones, info  = env.step(action)
        if dones:
           rospy.loginfo("Observation: %s, Reward: %f, Done: %s, Info: %s", obs, reward, dones, info)
   
        if dones:
            epi_count += 1
            rospy.logwarn("Episode: " + str(epi_count))
            obs = env.reset()
            # rospy.loginfo("Observation: %s, Reward: %f, Done: %s, Info: %s", obs, reward, dones, info)
    env.close()
    sys.exit()