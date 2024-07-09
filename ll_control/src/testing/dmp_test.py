#!/usr/bin/env python
import numpy as np
import rospy
import subprocess

from std_srvs.srv import Trigger
from ll_control.srv import *
from my_dmpbbo.my_dmpbbo import KulDMP
import utils.utils as utils


def run_DesiredCartesianNode():
    return_code = subprocess.call(['rosrun', 'll_control', 'DesiredCartesianNodePub.py'])
    if return_code != 0:
        print("An error occurred while running the rosrun command")
    
def clock_callback(msg):
    global current_time
    current_time = msg.data

if __name__ == '__main__': 
    print('hey')    
    rospy.init_node('your_node_name')  # Initialize your ROS node
    
    #rosrun_thread = threading.Thread(target=run_DesiredCartesianNode)
    #rosrun_thread.start()
        
    rospy.wait_for_service('/read_only_controller/get_end_effector_pose')
    rospy.wait_for_service('/ll_ctrl/cartesian_publisher/set_cartesian_desired_pose')
    rospy.wait_for_service('/ll_ctrl/cartesian_publisher/start_publishing')
    rospy.wait_for_service('/ll_ctrl/cartesian_publisher/stop_publishing')
    
    get_ef_pose = rospy.ServiceProxy('/read_only_controller/get_end_effector_pose', GetPose)
    set_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/set_cartesian_desired_pose', SetPose)
    start_pub_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/start_publishing', Trigger)
    stop_pub_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/stop_publishing', Trigger)
    
    
    
    goal = np.array([0.2, 0.4, 0.05])
    #goal = np.array([0.4, 0.2, 0.5])
    goal = np.array([0.5, -0.35, 0.5])

    current_pose = get_ef_pose()
    
    cur_pos, cur_quat = utils.NumpyfromGetPose(current_pose)
    print('current position', cur_pos)
    
    #this means the start of the trajectory
    init_time = rospy.Time.now().to_sec()
    #print(cur_pos)
    #cur_pos = np.array([0.0, 1.0, 0.0])
    
    dmp_kul = [KulDMP(cur_pos[i], init_time, goal[i], 25.0) for i in range(3)]
    #dmp_kul = [KulDMP(0.4, 0, 1, 10) for i in range(3)]
    #dmp_kul = KulDMP(0.4, 0.0, 1.0, 10.0)
    
    
    des_pos = cur_pos
    des_pose = SetPoseRequest()
    utils.SetPosefromNumpy(des_pos, cur_quat, des_pose)
    set_ef_pose(des_pose)
    start_pub_ef_pose()
    last_time = 0
    rate = rospy.Rate(100)
    
    # Main control loop
    j = 0
    while not rospy.is_shutdown() and j < 25*100:
        j = j + 1
        current_time = rospy.Time.now().to_sec() - init_time
        t = current_time  # Get current time in seconds
        dt = t - last_time
        dt = 1.0/100.0
        print('time', dt)
        for i in range(3):
            des_pos[i] = dmp_kul[i].integrate_step(dt, 'kuta')[1]
        #des_pos = dmp_kul.integrate_step(dt, 'kuta')
        print('i', j, des_pos)
        
        last_time = t
        
        utils.SetPosefromNumpy(des_pos, cur_quat, des_pose)
        set_ef_pose(des_pose)
        
        rate.sleep()  # Maintain loop rate
        
    stop_pub_ef_pose()
    #dmp_kul.plot_state_traj('kuta', False)
    for dmp in dmp_kul:
        dmp.plot_state_traj('kuta', True)
    
    import matplotlib.pyplot as plt
    input("Press ENTER to close all plots")    
    plt.close('all')
    # Optionally, stop publishing before exiting
