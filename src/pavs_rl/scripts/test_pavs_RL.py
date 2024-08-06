#!/usr/bin/env python

from pickle import POP
import sys
import gym
from frobs_rl.models.ppo import PPO
from frobs_rl.wrappers.NormalizeActionWrapper import NormalizeActionWrapper
from frobs_rl.wrappers.NormalizeObservWrapper import NormalizeObservWrapper
import rospy
import rospkg
from frobs_rl.common import ros_gazebo, ros_node
from frobs_rl.models.sac import SAC
from frobs_rl.models.td3 import TD3
from frobs_rl.wrappers.PavsActionWrapper import PavsActionWrapper
from frobs_rl.wrappers.PavsObservWrapper import PavsObservWrapper
from frobs_rl.wrappers.PavsTimeLimitWrapper import PavsTimeLimitWrapper
from gym import register

register(
    id='PavsReachAvoidEnv-v0',
    entry_point='pavs_env.pavs_reachavoid_env:PavsReachAvoidEnv',
    max_episode_steps=100,
)


if __name__ == '__main__':

   
    # ros_node.ros_kill_all_processes()gggg
  
    # ros_gazebo.launch_Gazebo(paused=True, gui=False)
  
    rospy.logwarn("Start")
    rospy.init_node('train_irb120_reacher')
    env = gym.make('PavsReachAvoidEnv-v0')


    #--- Normalize action space
    env = NormalizeActionWrapper(env)

    #--- Normalize observation space
    env = NormalizeObservWrapper(env)

    env.reset()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("pavs_rl")
    # rospy.loginfo("n_envs:"+env.num_envs)
    #模型将要保存的路径ff
    save_path = pkg_path + "/models/TD3/" 
    #日志将要保存的路径
    log_path = pkg_path + "/logs/TD3/"               # Path where the logs will be saved
    # model = SAC(env, save_path, log_path, config_file_pkg="pavs_rl", config_filename="sac_config.yaml")
    #model = PPO(env, save_path, log_path, config_file_pkg="pavs_rl", config_filename="ppo_config.yaml")
    model = TD3(env, save_path, log_path, config_file_pkg="pavs_rl", config_filename="td3_config.yaml")
    model.train()
    model.save_model()
    model.close_env()

    # ros_gazebo.gazebo_pause_physics()

    sys.exit()