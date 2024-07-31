#!/usr/bin/env python
import rospy

from hl_control.srv import StringIn
from transformer.srv import GetObjectPoseFromDescription, GetObjectPoseFromDescriptionRequest
from control.srv import SetDmp, SetDmpRequest


class InterfaceLLMsrvToLLsrv():
    #for now only input is the user string, other things must be set in the transformer
    #can change to more general conversation input by changing to CallLLM_long.srv, and 
    #bypassing the LLM node (transformer/rosmelodic)? 
    def __init__(self):
        rospy.init_node('hl_interface')
        

        
        self._mv_to_object_from_des_srv = rospy.Service('/hl_control/move_to_object_from_description', StringIn, self.move_to_object_from_description)
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
        
if __name__ == "__main__":
    try:
        a = InterfaceLLMsrvToLLsrv()
    except Exception as e:
            rospy.logerr('LLM node could not be initialized: {}'.format(e))
        
