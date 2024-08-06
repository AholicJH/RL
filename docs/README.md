# RL Platform with Gazebo and PAVS

## Purpose

The purpose of this repository is to develop a **robust Reinforcement Learning (RL) platform** that integrates the `Gazebo` simulation environment with the `frobs_rl` library. This platform will facilitate the development, training, and deployment of RL agents in both simulated and `real-world` environments.

This platform aims to:
- Provide a simulation environment using `Gazebo` for training RL agents.
- Utilize `frobs_rl` to implement and train state-of-the-art RL algorithms.
- Enable the transfer of trained agents from simulation to `real-world robotic systems`, specifically the [ISSPA](https://tis.ios.ac.cn/isspa/) platform.



## Based On

- **Gazebo**: A powerful robotics simulation tool.
- **frobs_rl**: A library for developing and training reinforcement learning algorithms.
- **ros-noetic (Robot Operating System)**: Middleware for robot software development.
- **Python 3.8 or higher**: Programming language for RL algorithm implementation, ros implementation and PAVS development.
- **ubuntu 20.04**: Operating System for development.
- **PAVS (Physical Agent Vehicle Small)**: Real-world small vehicle platform [ISSPA](https://tis.ios.ac.cn/isspa/).



## Required Features

1. **Gazebo Simulation Environment**
   - Set up Gazebo with various pre-configured worlds.
   - Integrate robot models and controllers.

2. **Reinforcement Learning Integration**
   - Implement RL algorithms using `frobs_rl`.
   - Set up training pipelines and evaluation metrics.

3. **Real-World Deployment**
   - Export trained models.
   - Adapt and deploy models to the PAVS platform.

4. **User Interface**
   - Provide a user-friendly interface for launching simulations, training processes, and deploying models.



## Installation

1. Install `ros-noetic` by following the [official installation](https://wiki.ros.org/noetic/Installation/Ubuntu).

2. Install following packages:
```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-navigation ros-noetic-gmapping ros-noetic-teb-local-planner ros-noetic-ackermann-msgs ros-noetic-gazebo-ros-pkgs ros-noetic-joint-state-publisher-gui ros-noetic-moveit xterm
```

3. [conda](https://docs.anaconda.com/free/miniconda/) is recommended to managing python environment. Follow these steps:
```bash
conda create -n gazebo_rl python=3.8
conda activate gazebo_rl
```

4. [frobs_rl](https://github.com/jmfajardod/frobs_rl) is required for training RL agents in gazebo, Follow these steps:
```bash
conda activate gazebo_rl
pip3 install -r requirements.txt
```

5. Build the project:
```
cd /path/to/gazebo_rl/
catkin build
```
6. Set up environments  
**For bash users:**
```bash
source devel/setup.bash
```
**For zsh users:**
```bash
source devel/setup.zsh
```



## TODO List

### Initial Setup

- [x] Set up Gazebo environment with ROS.
- [x] Create basic robot models in URDF.
- [x] Create basic lidar models.
- [x] Install and set up frobs_rl.

### Testing and reconfigure simulation model

- [ ] Validate simulation accuracy and realism.  
test robot model's dynamic and reconfigure robot model to make it simular to physical agents.
- [ ] Reconfigure the number of lasers for the lidar both in simulation and physical agent.

### Simulation and Training

- [ ] Develop training environments in Gazebo.
- [ ] Implement sample RL algorithms (e.g., DQN, PPO) using `frobs_rl`.
- [ ] Set up training configuration files.
- [ ] Implement reward structures and evaluation metrics.

### Testing and Validation

- [ ] Test RL agent training in various scenarios.
- [ ] Analyze training results and optimize parameters.

### Real-World Deployment

- [ ] Develop scripts for exporting trained models.
- [ ] Set up ROS nodes on the PAVS platform.
- [ ] Test and validate the deployment process.
- [ ] Fine-tune the model for real-world conditions.

### Documentation

- [ ] Create detailed user guides and tutorials.
- [ ] Document the API for extending the platform.
- [ ] Provide examples of successful training and deployment scenarios.



## Issues
See [issues](./ISSUES.md) for more details.

