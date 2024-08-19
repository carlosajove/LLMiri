#!/usr/bin/env python
import rospy

from hl_control.srv import StringIn
from transformer.srv import (GetObjectPoseFromDescription, GetObjectPoseFromDescriptionRequest,
                             GetCartesianStiffnessFromDescription, GetCartesianStiffnessFromDescriptionRequest)
from control.srv import SetDmp, SetDmpRequest, GraspObject, OpenGripper, WaitForGoal

from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter, GroupState

class InterfaceLLMsrvToLLsrv():
    #for now only input is the user string, other things must be set in the transformer
    #can change to more general conversation input by changing to CallLLM_long.srv, and 
    #bypassing the LLM node (transformer/rosmelodic)? 
    def __init__(self):
        rospy.init_node('hl_interface')
        self._mv_to_object_from_des_srv = rospy.Service('/hl_control/move_to_object_from_description', StringIn, self.move_to_object_from_description)
        self._pick_object_from_des_srv = rospy.Service('/hl_control/pick_object_from_description', StringIn, self.pick_object_from_description_handle)
        self._set_cart_stiff_from_des_srv = rospy.Service('/hl_control/set_cartesian_stiffness_from_description', StringIn, self.set_cartesian_stiffness_from_description_handle)
        rospy.spin()
        
    def move_to_object_from_description(self, req):
        """
        From a description of the object, the LLM interprets which object should be picked. And the robotic arm moves the end-effector to the object position.
        """
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
        """
        From the object description proviced by the user, the LLM tells what force and width parameters should be used to pick an object.
        """
        rospy.wait_for_service('/gripper_manager/open_gripper')
        open_grip_srv = rospy.ServiceProxy('/gripper_manager/open_gripper', OpenGripper)
        open_grip_srv(width=0.08, speed=0.1)
        
        rospy.wait_for_service('/hl_control/move_to_object_from_description')
        move_srv = rospy.ServiceProxy('/hl_control/move_to_object_from_description', StringIn)
        move_srv(req)
        
        rospy.wait_for_service('control/wait_for_ef_trajectory_goal')
        wait_srv = rospy.ServiceProxy('control/wait_for_ef_trajectory_goal', WaitForGoal)
        wait_srv(eps_pos = 0.1, eps_ori = 1)
        
        rospy.wait_for_service('/gripper_manager/grasp_object_from_description')
        grasp_srv = rospy.ServiceProxy('/gripper_manager/grasp_object_from_description', GraspObject)
        grasp_srv(req.user)
    
    
    def set_cartesian_stiffness_from_description_handle(self, req):
        """
        From the description provided by the user, the LLM sets the cartesiand stiffness used in the low-level cartesian impedance controller
        """
        rospy.wait_for_service('transformer/set_cartessian_stiffness_from_description')
        get_stiffness = rospy.ServiceProxy('transformer/set_cartessian_stiffness_from_description', GetCartesianStiffnessFromDescription)
        stiffness_response = get_stiffness(req.user)
        rospy.loginfo('Setting Controller Stiffness')
        request = ReconfigureRequest()
        
        request.config.bools = []
        request.config.ints = []
        request.config.strs = []
        request.config.doubles = [
            DoubleParameter(name='translational_stiffness', value=stiffness_response.translational_stiffness),
            DoubleParameter(name='rotational_stiffness', value=stiffness_response.rotational_stiffness)
        ]
        request.config.groups = [
            GroupState(name='LLM', state=True, id=0, parent=0)
        ]
        try:
            rospy.wait_for_service('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters') 
            controller_stiffness_client =  rospy.ServiceProxy('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters', Reconfigure)    
            response = controller_stiffness_client(request)
            print("Parameters updated successfully")
            return True
        except rospy.ServiceException as e:
            print("Service call failed %s" % e)
            return False
        
if __name__ == "__main__":
    try:
        a = InterfaceLLMsrvToLLsrv()
    except Exception as e:
            rospy.logerr('LLM node could not be initialized: {}'.format(e))
        
