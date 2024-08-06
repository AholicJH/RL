#!/usr/bin/env python

import os
import rospkg
import pandas as pd
import matplotlib.pyplot as plt

# 获取 ROS 包路径
rospack = rospkg.RosPack()
pkg_path = rospack.get_path("pavs_rl")

# 指定 TD3 保存路径
save_path_td3 = os.path.join(pkg_path, "logs", "td3", "TD3_1_31_07_2024_10_20_01", "progress.csv")
# 指定 PPO 保存路径
save_path_ppo = os.path.join(pkg_path, "logs", "ppo", "PPO_1_31_07_2024_14_49_36", "progress.csv")

# 读取 TD3 CSV 文件
data_td3 = pd.read_csv(save_path_td3)
# 读取 PPO CSV 文件
data_ppo = pd.read_csv(save_path_ppo)

# 提取需要的列
total_timesteps_td3 = data_td3["time/total_timesteps"]
ep_rew_mean_td3 = data_td3["rollout/ep_rew_mean"]

total_timesteps_ppo = data_ppo["time/total_timesteps"]
ep_rew_mean_ppo = data_ppo["rollout/ep_rew_mean"]

# 绘制图形
plt.figure(figsize=(10, 6))
plt.plot(total_timesteps_td3, ep_rew_mean_td3, label='TD3 Episode Reward Mean')
plt.plot(total_timesteps_ppo, ep_rew_mean_ppo, label='PPO Episode Reward Mean')
plt.xlabel('Total Timesteps')
plt.ylabel('Episode Reward Mean')
plt.title('Episode Reward Mean vs Total Timesteps')
plt.legend()
plt.grid(True)
plt.show()
