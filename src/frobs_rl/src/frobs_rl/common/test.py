import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

def switch_controllers(ns=None):
    if ns is not None:
        srv_name = ns + '/controller_manager/switch_controller'
    else:
        srv_name = '/controller_manager/switch_controller'
    
    rospy.wait_for_service(srv_name)
    try:
        client_srv = rospy.ServiceProxy(srv_name, SwitchController)
        rospy.loginfo(f"Service {srv_name} is available.")
        
        start_controllers = [
            'left_front_wheel_velocity_controller',
            'right_front_wheel_velocity_controller',
            'right_rear_wheel_velocity_controller',
            'left_rear_wheel_velocity_controller',
            'left_steering_hinge_position_controller',
            'right_steering_hinge_position_controller',
            'joint_state_controller'
        ]
        stop_controllers = []
        strictness = 1
        start_asap = False
        timeout = 3.0

        srv_request = SwitchControllerRequest(
            start_controllers=start_controllers,
            stop_controllers=stop_controllers,
            strictness=strictness,
            start_asap=start_asap,
            timeout=timeout
        )
        rospy.loginfo(f"Request: {srv_request}")

        resp = client_srv(srv_request)
        rospy.loginfo(f"Response: {resp}")

        if resp.ok:
            rospy.loginfo("Controllers switched successfully.")
        else:
            rospy.logwarn("Failed to switch controllers.")
            rospy.logwarn(f"SwitchControllerResponse: {resp}")
            # 增加具体的错误信息输出
            rospy.logwarn(f"Failed controllers: {start_controllers}")
            rospy.logwarn(f"Strictness: {strictness}")
            rospy.logwarn(f"Start ASAP: {start_asap}")
            rospy.logwarn(f"Timeout: {timeout}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('controller_switcher')
    switch_controllers('/pav_s00')  # 根据需要指定命名空间
