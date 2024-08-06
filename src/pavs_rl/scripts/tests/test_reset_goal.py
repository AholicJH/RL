#!/usr/bin/env python

import os
import math
import random
import rospy
import rospkg
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel
from frobs_rl.common import ros_gazebo, ros_node

<<<<<<< HEAD
=======

>>>>>>> 366d94b8108f8ad079f62cb7ebe99d75ed3273bd
if __name__ == '__main__':
    
    rospy.loginfo("Start")  
    rospy.init_node('test_reset_goal')
    
<<<<<<< HEAD
    # 等待Gazebo服务可用
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/delete_model')
    
    # 初始化服务代理
=======
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
>>>>>>> 366d94b8108f8ad079f62cb7ebe99d75ed3273bd
    reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    
    rospy.loginfo("Services are available")
    
<<<<<<< HEAD
    # 获取模型路径
=======
>>>>>>> 366d94b8108f8ad079f62cb7ebe99d75ed3273bd
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("pavs_rl")
    goal_model_path = os.path.join(pkg_path, 'scripts/pavs_env/sdf_model/goal.sdf')
    
    for i in range(10000):
        if i % 10 == 0:
<<<<<<< HEAD
            # 删除旧的目标模型
            try:
                delete_model_service(model_name='goal')
                rospy.loginfo("Old goal has been deleted")
            except rospy.ServiceException as e:
                rospy.logerr("Failed to delete goal: %s" % e)
                
            # 重置Gazebo世界
=======
            # reset empty world 
>>>>>>> 366d94b8108f8ad079f62cb7ebe99d75ed3273bd
            try:
                reset_world_service()
                rospy.loginfo("World has been reset")
            except rospy.ServiceException as e:
                rospy.logerr("Failed to reset world: %s" % e)
            
<<<<<<< HEAD
            # 随机生成目标位置
            r = 0.5
            max_distance = 5.0
            min_distance = 0.5  # 设定一个最小距离，以确保目标点不会生成在原点
            x_goal, y_goal = 0, 0

            while True:
                x_goal = random.uniform(-10 + 2 * r, 10 - 2 * r)
                y_goal = random.uniform(-10 + 2 * r, 10 - 2 * r)
                distance_from_origin = math.sqrt(x_goal**2 + y_goal**2)
                
                if min_distance < distance_from_origin <= max_distance:
                    break
            
            # 重置目标模型位置
            is_reset_goal = ros_gazebo.gazebo_reset_goal(goal_model_path, [x_goal, y_goal, 0.])

            if is_reset_goal:
                rospy.loginfo("Goal has been reset successfully")
            else:
                rospy.logerr("Failed to reset goal")
            
            # 打印倒计时
            for remaining_time in range(5, 0, -1):
                rospy.loginfo(f"Waiting... {remaining_time} seconds remaining")
                rospy.sleep(1)
    
    rospy.spin()
=======
            r = 0.5
            x_goal = random.uniform(-10 + 2* r, 10 - 2*r)
            y_goal = random.uniform(-10 + 2* r, 10 - 2*r)
            
            # reset goal 
            # is_reset_goal = ros_gazebo.gazebo_reset_goal([0., 0., 0.])
            is_reset_goal = ros_gazebo.gazebo_reset_goal(goal_model_path, [x_goal, y_goal, 0.])
            
            rospy.loginfo_once(f"Reset goal {is_reset_goal}")
            
            rospy.sleep(5)
    
    rospy.spin()
    
>>>>>>> 366d94b8108f8ad079f62cb7ebe99d75ed3273bd
