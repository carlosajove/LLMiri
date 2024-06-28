#!/usr/bin/env python


import rospy
import numpy as np
import subprocess
import threading
from copy import copy


import utils

from std_srvs.srv import Trigger
from ll_control.srv import *




def run_DesiredCartesianNode():
    return_code = subprocess.call(['rosrun', 'll_control', 'DesiredCartesianNodePub.py'])
    if return_code != 0:
        print("An error occurred while running the rosrun command")



class moveEndEff():
    def __init__(self):
        
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
        
        self._curr_ef_pose = get_ef_pose()
        self._init_ef_pose = copy(self._curr_ef_pose)

        self._curr_ef_pos, self._curr_ef_quat = utils.GetPosetoVectors(self._curr_ef_pose)
    
    def 












class line_trajectory():
    def __init__(self, start_pos, end_pos, duration):
        self._start_pos = copy(start_pos)
        self._end_pos = copy(end_pos)
        if duration == 0:
            print("[ERROR] Duration  = 0. INCRESE DURATION")
        self._duration = copy(duration)
        
        
    def compute(self, t):
        if t > self._duration:
            return self._end_pos
        res = self._end_pos - self._start_pos
        res /= self._duration
        res *= t
        res += self._start_pos
        return res
    
    
def clock_callback(msg):
    global current_time
    current_time = msg.data

if __name__ == '__main__':     

    
    
    
    goal = np.array([0.5, 0.33, 0.6])
    current_pose = get_ef_pose()
    cur_pos, cur_quat = utils.GetPosetoVectors(current_pose)


    
    traj = line_trajectory(cur_pos, goal, 5) #duration 5 seconds
    
    #this means the start of the trajectory
    init_time = rospy.Time.now()
    des_pos = traj.compute(0)
    des_pose = SetPoseRequest()
    utils.VectorstoSetPose(des_pos, cur_quat, des_pose)
    set_ef_pose(des_pose)
    start_pub_ef_pose()
    
    rate = rospy.Rate(100)
    
    # Main control loop
    while not rospy.is_shutdown():
        current_time = rospy.Time.now() - init_time
        t = current_time.to_sec()  # Get current time in seconds
        
        des_pos = traj.compute(t)
        print(des_pos)
        utils.VectorstoSetPose(des_pos, cur_quat, des_pose)
        set_ef_pose(des_pose)
        
        rate.sleep()  # Maintain loop rate

    # Optionally, stop publishing before exiting
    stop_pub_ef_pose()