import rospy
import numpy as np
from copy import copy,deepcopy

from my_dmpbbo.my_dmpbbo import KulDMP
import utils.utils as utils

from std_srvs.srv import Trigger
from control.srv import *
from geometry_msgs.msg import Pose



class efTrajectory():
    def __init__(self, rate):
        rospy.wait_for_service('/read_only_controller/get_end_effector_pose')
        rospy.wait_for_service('/ll_ctrl/cartesian_publisher/set_cartesian_desired_pose')
        rospy.wait_for_service('/ll_ctrl/cartesian_publisher/start_publishing')
        rospy.wait_for_service('/ll_ctrl/cartesian_publisher/stop_publishing')
        
        self._get_ef_pose = rospy.ServiceProxy('/read_only_controller/get_end_effector_pose', GetPose)
        self._set_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/set_cartesian_desired_pose', SetPose)
        self._start_pub_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/start_publishing', Trigger)
        self._stop_pub_ef_pose = rospy.ServiceProxy('/ll_ctrl/cartesian_publisher/stop_publishing', Trigger)
        
        
        self.set_goal(np.concatenate(utils.NumpyfromGetPose(self._get_ef_pose())))
        self._rate = rate
        
    def set_goal(self, new_goal):
        if not isinstance(new_goal, np.ndarray):
            raise TypeError("new_goal must be a numpy array")
        if new_goal.shape != (6,):
            raise ValueError("new_goal must be a 6-dimensional numpy array")
        self._goal = copy(new_goal)
    
    def set_duration(self, tau):
        self._tau = copy(tau)
        
    
        
        
    def generate_DMPtrajectory(self):
        init_pos, init_ori = utils.NumpyfromGetPose(self._get_ef_pose())
        
        self._dmp_kul = [KulDMP(init_pos[i], 0.0, self._goal[i], self._tau) for i in range(3)]

    
    def update_loop(self):
        dt = 1.0/self._rate
        des_pose  = Pose()
        des_pos = np.zeros(3)
        j = 0
        while j < self._rate*self._tau:
            for i in range(3):
                des_pos[i] = self._dmp_kul[i].integrate_step(dt, 'kuta')[1]    
            
            

            #compute ori
            des_quat = np.zeros(4)
            des_quat[3] = 1
            
            utils.SetPosefromNumpy(des_pos, des_quat, des_pose)
            self._set_ef_pose(des_pose)
        
        
        
        
        
    
        
        
        
        
    
    
    
    