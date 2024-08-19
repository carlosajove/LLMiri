#!/usr/bin/env python
import rospy

from franka_gripper.msg import MoveActionGoal, GraspActionGoal
from control.srv import GraspObject, OpenGripper
from transformer.srv import GetGripperForceFromDescription

class GripperManager():
    def __init__(self,):
        rospy.init_node('gripper_manager')
        
        self._grasp_object_srv = rospy.Service('gripper_manager/grasp_object_from_description', GraspObject, self.grasp_object_handle)
        self._grasp_object_publisher = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=2)
        
        self._open_gripper_srv = rospy.Service('gripper_manager/open_gripper', OpenGripper, self.open_gripper_handle)
        self._open_gripper_publisher = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=2)
        
        rospy.spin()
        
                
    def grasp_object_handle(self, req):
        """
        Uses LLM to set the width and force of the gripper
        """
        rospy.wait_for_service('/transformer/get_gripper_force_from_description')
        self._get_gripper_force_srv = rospy.ServiceProxy('/transformer/get_gripper_force_from_description', GetGripperForceFromDescription)
        
        
        gripper_force_response = self._get_gripper_force_srv(req.description)
        
        pub_msg = GraspActionGoal()
        pub_msg.goal.width = gripper_force_response.width
        pub_msg.goal.force = gripper_force_response.force
        pub_msg.goal.epsilon.inner = 0.005
        pub_msg.goal.epsilon.outer = 0.005
        pub_msg.goal.speed = 0.1
        
        rospy.loginfo('Publishing grasp object')
        self._grasp_object_publisher.publish(pub_msg)
        
        return True
        
    def open_gripper_handle(self, req):
        pub_msg = MoveActionGoal()
        pub_msg.goal.width = req.width
        pub_msg.goal.speed = req.speed 
        
        self._open_gripper_publisher.publish(pub_msg)
        return True
    
    
if __name__ == "__main__":
    try:
        GripperManager()
    except rospy.ROSInterruptException:
        rospy.loginfo('Error Gripper Manager Node initialisation')