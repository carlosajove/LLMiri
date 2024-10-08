#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import quaternion


from control.srv import SetDmp, GetPose, SetInt, WaitForGoal
from geometry_msgs.msg import PoseStamped

from my_dmpbbo.my_dmpbbo import MultiKulDmp
from my_dmpbbo.dmp_ori import OriDmp

class EndEffectorTrajectoryNode():
    def __init__(self):
        rospy.init_node('ef_traj_node')
        
        
        self._start_pos_traj_srv = rospy.Service('control/start_pos_trajectory', SetDmp, self.start_pos_traj_handle)
        self._start_pos_traj_trigger = False
        self._change_pos_goal_srv = rospy.Service('control/change_pos_goal', SetDmp, self.change_pos_goal_handle)
        
        self._start_ori_traj_srv = rospy.Service('control/start_ori_trajectory', SetDmp, self.start_ori_traj_handle)
        self._start_ori_traj_trigger = False
        self._change_ori_goal_srv = rospy.Service('control/change_ori_goal', SetDmp, self.change_ori_goal_handle)
        
        self._wait_for_goal_srv = rospy.Service('control/wait_for_ef_trajectory_goal', WaitForGoal, self.wait_for_goal_handle)
        
        self._update_freq = 1000 #Franka c++ interface can work at 1 kHz
        self._set_update_freq_srv = rospy.Service('control/trajectory_update_frequency', SetInt, self.set_update_freq_handle)
        
        self._pos_traj = None
        self._ori_traj = None
        self._trajectory_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)  

        rospy.loginfo('ef_node initialised')
        self.publish_trajectory()
        rospy.spin()


    def set_update_freq_handle(self, req):
        self._update_freq = req.data
        return True
    
    def start_pos_traj_handle(self, req):
        rospy.wait_for_service('/read_only_controller/get_end_effector_pose')
        ef_pose_srv = rospy.ServiceProxy('/read_only_controller/get_end_effector_pose', GetPose)
        ef_pose = ef_pose_srv()
        init_pos = np.array([ef_pose.position_x, ef_pose.position_y, ef_pose.position_z])
        goal = np.array([req.pose.position.x, req.pose.position.y, req.pose.position.z])

        self._pos_traj = MultiKulDmp(init_pos=init_pos, 
                                     init_time=0, 
                                     goal=goal, 
                                     tau=req.tau)
        self._start_pos_traj_trigger = True
        self._pos_traj_start_time = rospy.get_time()
        rospy.loginfo('starting pos traj')
        return True
    
    def change_pos_goal_handle(self, req):
        if (self._start_pos_traj_trigger):
            goal = [req.pose.position.x, req.pose.position.y, req.pose.position.z]
            self._pos_traj.set_goal(goal)
            rospy.loginfo('New position goal set')
            return True
        else:
            rospy.loginfo('Position Trajectory is not defined')
            return False
    
    def start_ori_traj_handle(self, req):
        rospy.wait_for_service('/read_only_controller/get_end_effector_pose')
        ef_pose_srv = rospy.ServiceProxy('/read_only_controller/get_end_effector_pose', GetPose)
        ef_pose = ef_pose_srv()
        init_quat = [ef_pose.orientation_w, ef_pose.orientation_x, ef_pose.orientation_y, ef_pose.orientation_z]
        goal_quat = [req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z]
        
        self._ori_traj = OriDmp(ini_quat=init_quat, 
                                ini_time=0, 
                                goal_quat=goal_quat,
                                tau=req.tau)
        self._start_ori_traj_trigger = True
        self._ori_traj_start_time = rospy.get_time()
        rospy.loginfo('starting ori traj')

        return True
        
    def change_ori_goal_handle(self, req):
        if (self._start_pos_traj_trigger):
            goal = [req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z]
            self._ori_traj.set_goal(goal)
            rospy.loginfo('New quaternion goal set')
            return True
        else:
            rospy.loginfo('Quaternion Trajectory is not defined')
            return False
           
    def publish_trajectory(self):
        rate = rospy.Rate(self._update_freq)
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            if self._start_pos_traj_trigger and self._pos_traj:
                #if current_time - self._pos_traj_start_time <= self._pos_traj.get_tau():
                pos_traj_point = self._pos_traj.integrate_step(1.0/self._update_freq, 'kuta')
                """
                else:
                    self._start_pos_traj_trigger = False
                    self._pos_traj = None
                """
            if self._start_ori_traj_trigger and self._ori_traj:
                #if current_time - self._ori_traj_start_time <= self._ori_traj.get_tau():
                ori_traj_point = self._ori_traj.integrateOriInternal(1.0/self._update_freq)
                """
                else:
                    self._start_ori_traj_trigger = False
                    self._ori_traj = None
                """
            
            if self._start_pos_traj_trigger or self._start_ori_traj_trigger:
                ef_pose = PoseStamped()
                if self._pos_traj:
                    ef_pose.pose.position.x = pos_traj_point[0]
                    ef_pose.pose.position.y = pos_traj_point[1]
                    ef_pose.pose.position.z = pos_traj_point[2]
                if self._ori_traj:
                    #TODO:change to smart orientation
                    pass
                    """
                    ef_pose.pose.orientation.w = ori_traj_point.w
                    ef_pose.pose.orientation.x = ori_traj_point.x
                    ef_pose.pose.orientation.y = ori_traj_point.y
                    ef_pose.pose.orientation.z = ori_traj_point.z
                    """

                self._trajectory_pub.publish(ef_pose)
            
            rate.sleep()

    def wait_for_goal_handle(self, req):
        rospy.wait_for_service('/read_only_controller/get_end_effector_pose')
        ef_pose_getter = rospy.ServiceProxy('read_only_controller/get_end_effector_pose', GetPose)
        
        rate = rospy.Rate(5)
        pos_bool = False
        ori_bool = False
        while not pos_bool and not ori_bool:
            pos_bool = False
            ori_bool = False
            current_pose = ef_pose_getter()
            current_position = np.array([current_pose.position_x, 
                                         current_pose.position_y, 
                                         current_pose.position_z])            
            current_orientation = np.quaternion(current_pose.orientation_w,
                                                current_pose.orientation_x, 
                                                current_pose.orientation_y, 
                                                current_pose.orientation_z)
            if self._pos_traj:
                pos_goal = self._pos_traj.get_goal()
                pos_goal = np.array(pos_goal)
                if np.linalg.norm(current_position - pos_goal) < req.eps_pos:
                    pos_bool = True
                    rospy.loginfo('Wait for goal srv: Pos goal reached')
            else:
                rospy.loginfo('Wait for goal srv: Pos traj not set')
                pos_bool = True
                
            if self._ori_traj:
                ori_goal = self._ori_traj.get_goal()
                ori_goal = ori_goal.normalized()
                current_orientation = current_orientation.normalized()
                
                ori_goal_inv = ori_goal.inverse()
                q_rel = current_orientation * ori_goal_inv
                error_angle = 2 * np.arccos(np.clip(q_rel.w, -1.0, 1.0))
                if error_angle < req.eps_ori:
                    ori_bool = True
                    rospy.loginfo('Wait for goal srv: Ori goal reached')
                
            else:
                rospy.loginfo('Wait for goal srv: Ori traj not set')
                ori_bool = True

                
        return True

        """
        
            current_position = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
    
    # Extract target position and orientation
    target_position = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    target_orientation = [target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]

    # Calculate Euclidean distance for position
    position_distance = np.linalg.norm(current_position - target_position)
    
    # Calculate quaternion distance for orientation
    quat_diff = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_inverse(current_orientation),
        target_orientation
    )
    angle_diff = 2 * np.arccos(np.clip(quat_diff[3], -1.0, 1.0))

    # Check if both position and orientation distances are within the thresholds
    return position_distance < pos_threshold and angle_diff < orient_threshold

        """


        
if __name__ == '__main__':
    try:
        EndEffectorTrajectoryNode()
    except rospy.ROSInterruptException:
        rospy.loginfo('Error End Effector Trajectory Node initialisation')