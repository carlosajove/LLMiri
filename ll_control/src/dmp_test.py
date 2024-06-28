#!/usr/bin/env python



import numpy as np
import rospy
import threading
import subprocess
import utils


from dmp_v00 import *
from std_srvs.srv import Trigger
from ll_control.srv import *

def run_DesiredCartesianNode():
    return_code = subprocess.call(['rosrun', 'll_control', 'DesiredCartesianNodePub.py'])
    if return_code != 0:
        print("An error occurred while running the rosrun command")
    
def clock_callback(msg):
    global current_time
    current_time = msg.data


from my_dmpbbo.my_dmpbbo import KulDMP


if __name__ == '__main__':     
    rospy.init_node('your_node_name')  # Initialize your ROS node
    
    rosrun_thread = threading.Thread(target=run_DesiredCartesianNode)
    rosrun_thread.start()
        
    rospy.wait_for_service('/read_only_controller/get_end_effector_pose')
    rospy.wait_for_service('/ll_ctrl/cartesian_publisher/set_cartesian_desired_pose')
    rospy.wait_for_service('/ll_ctrl/cartesian_publisher/start_publishing')
    rospy.wait_for_service('/ll_ctrl/cartesian_publisher/stop_publishing')
    
    get_ef_pose = rospy.ServiceProxy('/read_only_controller/get_end_effector_pose', GetPose)
    set_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/set_cartesian_desired_pose', SetPose)
    start_pub_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/start_publishing', Trigger)
    stop_pub_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/stop_publishing', Trigger)
    
    
    
    goal = np.array([0.5, 0.33, 0.6])
    current_pose = get_ef_pose()
    cur_pos, cur_quat = utils.NumpyfromGetPose(current_pose)

    
    #this means the start of the trajectory
    init_time = rospy.Time.now().to_sec()
    dmp_kul = [KulDMP(cur_pos[i], init_time, goal[i], 5) for i in range(3)]
        
    des_pos = cur_pos
    des_pose = SetPoseRequest()
    utils.SetPosefromNumpy(des_pos, cur_quat, des_pose)
    set_ef_pose(des_pose)
    start_pub_ef_pose()
    last_time = 0
    rate = rospy.Rate(100)
    
    # Main control loop
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec() - init_time
        t = current_time  # Get current time in seconds
        dt = t - last_time
        
        for i in range(3):
            des_pos[i] = dmp_kul[i].integrate_step(dt, 'kuta')[1]
             
        
        last_time = t
        
        print(des_pos)
        utils.SetPosefromNumpy(des_pos, cur_quat, des_pose)
        set_ef_pose(des_pose)
        
        rate.sleep()  # Maintain loop rate

    # Optionally, stop publishing before exiting
    stop_pub_ef_pose()