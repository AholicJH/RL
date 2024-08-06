#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import rospy
import random #随机
from gazebo_msgs.srv import SpawnModel, DeleteModel,GetWorldProperties
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion

def load_model(model_path):
    with open(model_path, 'r') as model_file:
        model_xml = model_file.read()
    return model_xml

def spawn_model(model_name, model_xml, pose, reference_frame="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(model_name, model_xml, "", pose, reference_frame)
        rospy.loginfo("Model {} spawned successfully".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_prox(model_name)
        rospy.loginfo("Model {} deleted successfully".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def get_all_model_names():
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties_prox = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        response = get_world_properties_prox()
        model_names = response.model_names
        rospy.loginfo("Current models in the world: {}".format(model_names))
        return model_names
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return []

def reset_environment():
    rospy.loginfo("Resetting environment to initial state.")
    model_names = get_all_model_names()
    for model_name in model_names:
        delete_model(model_name)
    rospy.loginfo("Environment reset complete.")

if __name__ == "__main__":
    rospy.init_node('spawn_random_models_node')

    model_name_prefix = "cafe_table_"
    model_path = "/home/jianghan/.gazebo/models/cafe_table/model.sdf"
    model_xml = load_model(model_path)
    num_models = 10
    delay = 30  # 延迟时间，单位为秒
    # 地图边界定义（
    x_min, x_max = -10, 10
    y_min, y_max = -10, 10

    # 存储生成的模型名称
    # model_names = []

    # 生成随机位置并生成模型
    for i in range(num_models):
        model_name = model_name_prefix + str(i)
        pose = Pose()
        pose.position = Point(
            x=random.uniform(x_min, x_max),
            y=random.uniform(y_min, y_max),
            z=0
        )
        pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

        spawn_model(model_name, model_xml, pose)
        # model_names.append(model_name)  # 记录生成的模型名称
    
    # 等待1分钟
    # rospy.loginfo("Waiting {} seconds before deleting models".format(delay))
    rospy.loginfo("Waiting {} seconds before resetting environment".format(delay))
    time.sleep(delay)
        

    # for i in range(num_models):
    #     model_name = model_name_prefix + str(i)
    #     # delete_model(model_name)
    # 删除所有生成的模型
    # for model_name in model_names:
    #     delete_model(model_name)
    reset_environment()