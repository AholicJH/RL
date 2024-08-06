#!/usr/bin/env python

import os
import random
from frobs_rl.common.ros_gazebo import gazebo_load_sdf
import rospy
import rospkg
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def get_random_position(exclude_positions, min_distance, x_range=(-10, 10), y_range=(-10, 10)):
    """
    Generate a random position within the specified range that is at least `min_distance` away from excluded positions.
    
    :Param
    -------
        - exclude_positions: List of positions to exclude.
        - min_distance: Minimum distance from any excluded position.
        - x_range: Range for x-coordinate.
        - y_range: Range for y-coordinate.
        
    :Return
    -------
        - position: A valid random position.
    """
    while True:
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        position = (x, y, 0.0)
        
        if all(((pos[0] - position[0]) ** 2 + (pos[1] - position[1]) ** 2) ** 0.5 >= min_distance for pos in exclude_positions):
            return position

def reset_goal_and_obstacles(goal_model_path, obstacle_model_path, goal_position, obstacle_positions, retries=5):
    """
    Function to reset goal and obstacles positions.
    
    :Param
    -------
        - goal_model_path: The path of goal sdf model.
        - obstacle_model_path: The path of obstacle sdf model.
        - goal_position: The position to place the goal.
        - obstacle_positions: List of positions to place the obstacles.
        - retries: The number of times to retry the service call.
        
    :Return
    -------
        - is_reset(bool): True if both models were reset successfully.
    """
    is_reset = False
    is_goal_delete = False
    is_goal_spawn = False
    obstacle_names = [f"obstacle_{i}" for i in range(len(obstacle_positions))]
    
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/delete_model')
    spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    
    goal_xml = gazebo_load_sdf(goal_model_path)
    obstacle_xml = gazebo_load_sdf(obstacle_model_path)
    
    goal_pose = Pose(position=Point(x=goal_position[0], y=goal_position[1], z=goal_position[2]), 
                     orientation=Quaternion(x=0, y=0, z=0, w=1))
    
    for retry in range(retries):
        if not is_goal_delete:
            try:
                delete_model_srv("goal")
                rospy.loginfo("Goal model has been deleted.")
                is_goal_delete = True
                rospy.sleep(1)  # 添加延时确保状态更新
            except rospy.ServiceException as e:
                rospy.logerr(f"Delete Goal Model service call failed: {e}")
                
        if not is_goal_spawn:
            try:
                spawn_model_srv("goal", goal_xml, "", goal_pose, "world")
                rospy.loginfo("Goal model has been spawned.")
                is_goal_spawn = True
            except rospy.ServiceException as e:
                rospy.logerr(f"Spawn Goal Model service call failed: {e}")
        
        obstacle_reset_success = True
        for i, pos in enumerate(obstacle_positions):
            obstacle_name = obstacle_names[i]
            obstacle_pose = Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]), 
                                 orientation=Quaternion(x=0, y=0, z=0, w=1))
            
            if not is_goal_delete:  # Reset obstacles only if goal is reset successfully
                try:
                    delete_model_srv(obstacle_name)
                    rospy.loginfo(f"Obstacle model {obstacle_name} has been deleted.")
                    rospy.sleep(1)  # 添加延时确保状态更新
                except rospy.ServiceException as e:
                    rospy.logerr(f"Delete Obstacle Model service call failed: {e}")
                    obstacle_reset_success = False
                    
            if obstacle_reset_success:
                try:
                    spawn_model_srv(obstacle_name, obstacle_xml, "", obstacle_pose, "world")
                    rospy.loginfo(f"Obstacle model {obstacle_name} has been spawned.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Spawn Obstacle Model service call failed: {e}")
                    obstacle_reset_success = False
        
        if is_goal_spawn and obstacle_reset_success:
            is_reset = True
            break
    
    return is_reset

if __name__ == '__main__':
    
    rospy.loginfo("Start")  
    rospy.init_node('test_reset_goal_and_obstacles')
    
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    
    rospy.loginfo("Services are available")
    
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("pavs_rl")
    goal_model_path = os.path.join(pkg_path, 'scripts/pavs_env/sdf_model/goal.sdf')
    obstacle_model_path = os.path.join(pkg_path, 'scripts/pavs_env/sdf_model/obstacle_static_box.sdf')
    
    exclude_positions = []  # List to keep track of previously used positions
    
    for i in range(10000):
        if i % 10 == 0:
            # reset empty world 
            try:
                reset_world_service()
                rospy.loginfo("World has been reset")
            except rospy.ServiceException as e:
                rospy.logerr("Failed to reset world: %s" % e)
            
            # Generate new positions for goal and obstacles
            goal_position = get_random_position(exclude_positions, min_distance=4.0)
            obstacle_positions = []
            for _ in range(5):  # Generate 5 obstacles
                pos = get_random_position([goal_position] + obstacle_positions, min_distance=4.0)
                obstacle_positions.append(pos)
            
            # Update exclude positions
            exclude_positions = [goal_position] + obstacle_positions
            
            # Reset goal and obstacles
            is_reset = reset_goal_and_obstacles(goal_model_path, obstacle_model_path, goal_position, obstacle_positions)
            
            rospy.loginfo_once(f"Reset goal and obstacles: {is_reset}")
            
            # Print countdown
            for remaining_time in range(10, 0, -1):
                rospy.loginfo(f"Waiting... {remaining_time} seconds remaining")
                rospy.sleep(1)
                
    rospy.spin()
