#!/usr/bin/env python

import os
import rospkg
import pandas as pd
import matplotlib.pyplot as plt

# 获取 ROS 包路径
rospack = rospkg.RosPack()
pkg_path = rospack.get_path("pavs_rl")

# 指定保存路径
save_path = os.path.join(pkg_path, "logs", "td3", "TD3_1_31_07_2024_10_20_01", "progress.csv")

# 读取 CSV 文件
data = pd.read_csv(save_path)

# 提取需要的列
total_timesteps = data["time/total_timesteps"]
ep_rew_mean = data["rollout/ep_rew_mean"]

# 计算滚动标准偏差
window_size = 100  # 你可以根据需要调整窗口大小
ep_rew_std = ep_rew_mean.rolling(window=window_size, min_periods=1).std()

# 绘制图形
plt.figure(figsize=(10, 6))
plt.plot(total_timesteps, ep_rew_mean, label='Episode Reward Mean')
plt.fill_between(total_timesteps, 
                 ep_rew_mean - ep_rew_std, 
                 ep_rew_mean + ep_rew_std, 
                 alpha=0.2, label='Rolling Std Deviation')
plt.xlabel('Total Timesteps')
plt.ylabel('Episode Reward Mean')
plt.title('Episode Reward Mean vs Total Timesteps')
plt.legend()
plt.grid(True)
plt.show()
