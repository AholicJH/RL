#!/usr/bin/env python

import os
import math
import random
import rospy
import rospkg
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel
from frobs_rl.common import ros_gazebo, ros_node


if __name__ == '__main__':
    
    rospy.loginfo("Start")  
    rospy.init_node('test_reset_obstacles')
    
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    
    rospy.loginfo("Services are available")
    
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("pavs_rl")
    goal_model_path = os.path.join(pkg_path, 'scripts/pavs_env/sdf_model/obstacle_static_box.sdf')
    
    for i in range(10000):
        if i % 10 == 0:
            # reset empty world 
            try:
                reset_world_service()
                rospy.loginfo("World has been reset")
            except rospy.ServiceException as e:
                rospy.logerr("Failed to reset world: %s" % e)
            
            r = 0.5
            x_goal = random.uniform(-10 + 2* r, 10 - 2*r)
            y_goal = random.uniform(-10 + 2* r, 10 - 2*r)
            
            # reset goal 
            # is_reset_goal = ros_gazebo.gazebo_reset_goal([0., 0., 0.])
            is_reset_goal = ros_gazebo.gazebo_reset_goal(goal_model_path, [x_goal, y_goal, 0.])
            
            rospy.loginfo_once(f"Reset obstacles {is_reset_goal}")
            
<<<<<<< HEAD
             # 打印倒计时
            for remaining_time in range(10, 0, -1):
                rospy.loginfo(f"Waiting... {remaining_time} seconds remaining")
                rospy.sleep(1)
=======
            rospy.sleep(5)
    
>>>>>>> 366d94b8108f8ad079f62cb7ebe99d75ed3273bd
    rospy.spin()
    