#!/usr/bin/env python


# Kill all processes related to previous runs
import sys
import gym
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
    max_episode_steps=1000,
)


if __name__ == '__main__':

   
    # ros_node.ros_kill_all_processes()
  
    # ros_gazebo.launch_Gazebo(paused=False, gui=False)
  
    rospy.logwarn("Start")
    rospy.init_node('train_irb120_reacher')

   
    env = gym.make('PavsReachAvoidEnv-v0')

   
    env = PavsActionWrapper(env)

  
    env = PavsObservWrapper(env)

   
    env = PavsTimeLimitWrapper(env, max_steps=100)
    env.reset()

    # 保存设置保存和日志路径
    # rospack = rospkg.RosPack()
    # pkg_path = rospack.get_path("pavs_rl")
    # rospy.loginfo("pkg_path:"+pkg_path)
    
    # 选择训练模型
    # save_path = pkg_path + "/models/sac/" # Path where the model will be saved
    # log_path = pkg_path + "/logs/sac/"               # Path where the logs will be saved
    # model = SAC(env, save_path, log_path, config_file_pkg="pavs_rl", config_filename="sac_config.yaml")
    
    # 模型的训练以及保存
    # model.train()

    # 测试模型
    #-- Test the model
    # obs = env.reset()
    # for _ in range(1000):
    #     action, _states = model.predict(obs)
    #     obs, rewards, dones, info = env.step(action)
    #     env.render()
    #     if dones:
    #         obs = env.reset()
    # model.save_model()
    # model.close_env()

    sys.exit()