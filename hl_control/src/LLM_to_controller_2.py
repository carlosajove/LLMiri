#!/usr/bin/env python
import rospy

from hl_control.srv import StringIn
from transformer.srv import GetObjectPoseFromDescription, GetObjectPoseFromDescriptionRequest
from control.srv import SetDmp, SetDmpRequest, GraspObject, OpenGripper, WaitForGoal


class InterfaceLLMsrvToLLsrv():
    #for now only input is the user string, other things must be set in the transformer
    #can change to more general conversation input by changing to CallLLM_long.srv, and 
    #bypassing the LLM node (transformer/rosmelodic)? 
    def __init__(self):
        rospy.init_node('hl_interface')
        self._mv_to_object_from_des_srv = rospy.Service('/hl_control/move_to_object_from_description', StringIn, self.move_to_object_from_description)
        self._pick_object_from_des_srv = rospy.Service('/hl_control/pick_object_from_description', StringIn, self.pick_object_from_description_handle)
        rospy.spin()
        
    def move_to_object_from_description(self, req):
        #TODO: Think what to do with tau
        rospy.wait_for_service('/transformer/get_object_pose_from_description')
        rospy.wait_for_service('/control/start_pos_trajectory')
        rospy.wait_for_service('/control/start_ori_trajectory')
        
        get_object_pose_from_description_srv = rospy.ServiceProxy('transformer/get_object_pose_from_description', GetObjectPoseFromDescription)
        start_pos_traj_srv = rospy.ServiceProxy('/control/start_pos_trajectory', SetDmp)
        start_ori_traj_srv = rospy.ServiceProxy('/control/start_ori_trajectory', SetDmp)
        
        get_obj_pose_from_des_response = get_object_pose_from_description_srv(req.user)
        start_pos_traj_req = SetDmpRequest()
        start_pos_traj_req.pose = get_obj_pose_from_des_response.pose
        start_pos_traj_req.twist = get_obj_pose_from_des_response.twist
        start_pos_traj_req.tau = 15.0
        start_ori_traj_req = start_pos_traj_req
        
        b1 = start_pos_traj_srv(start_pos_traj_req)
        b2 = start_ori_traj_srv(start_ori_traj_req)
        if not b1.success or not b2.success:
            return False
        return True
    
    def pick_object_from_description_handle(self, req):
        rospy.wait_for_service('/gripper_manager/open_gripper')
        open_grip_srv = rospy.ServiceProxy('/gripper_manager/open_gripper', OpenGripper)
        print(open_grip_srv(width=0.08, speed=0.1))
        
        rospy.wait_for_service('/hl_control/move_to_object_from_description')
        move_srv = rospy.ServiceProxy('/hl_control/move_to_object_from_description', StringIn)
        print(move_srv(req))
        
        rospy.wait_for_service('control/wait_for_ef_trajectory_goal')
        wait_srv = rospy.ServiceProxy('control/wait_for_ef_trajectory_goal', WaitForGoal)
        wait_srv(eps_pos = 0.1, eps_ori = 1)
        
        rospy.wait_for_service('/gripper_manager/grasp_object_from_description')
        grasp_srv = rospy.ServiceProxy('/gripper_manager/grasp_object_from_description', GraspObject)
        print(grasp_srv(req.user))
        
        
if __name__ == "__main__":
    try:
        a = InterfaceLLMsrvToLLsrv()
    except Exception as e:
            rospy.logerr('LLM node could not be initialized: {}'.format(e))
        
