#!/usr/bin/env python

#include <franka_gazebo/franka_hw_sim.h>
#hardware_interface::JointStateInterface

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from ll_control.srv import SetPose, SetPoseResponse
from std_srvs.srv import Trigger, TriggerResponse
import threading

from copy import copy, deepcopy

from utils import *

class CartesianPublisherNode():
    def __init__(self):
        rospy.init_node('desired_cartesian_pose_publisher', anonymous=True)
        
        self._set_cartesian_desired_srv = rospy.Service('/ll_ctrl/cartesian_publisher/set_cartesian_desired_pose', SetPose, self.handle_set_pos_command)
        self._start_publishing_srv = rospy.Service('/ll_ctrl/cartesian_publisher/start_publishing', Trigger, self.handle_start_publishing)
        self._stop_publishing_srv = rospy.Service('/ll_ctrl/cartesian_publisher/stop_publishing', Trigger, self.handle_stop_publishing)
        
        self._pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
        self._rate = rospy.Rate(1000)
        
        self._cartesian_pose = Pose()
        PosefromNumpy(np.zeros(3), np.zeros(4), self._cartesian_pose)
        print(self._cartesian_pose)
        
        self._initialised_pose = False
        self._is_publishing = False
        
        rospy.spin()

    def handle_set_pos_command(self, req):
        del self._cartesian_pose 
        self._cartesian_pose = Pose()

        self._cartesian_pose.position.x = req.position_x
        self._cartesian_pose.position.y = req.position_y
        self._cartesian_pose.position.z = req.position_z
        self._cartesian_pose.orientation.x = req.orientation_x
        self._cartesian_pose.orientation.y = req.orientation_y
        self._cartesian_pose.orientation.z = req.orientation_z
        self._cartesian_pose.orientation.w = req.orientation_w

        response = SetPoseResponse()
        response.success = True
        self._initialised_pose = True
        return response


    def handle_start_publishing(self,req):
        response = TriggerResponse()
        if (self._initialised_pose):
            self._is_publishing = True
            
            response.success = True
            response.message = "Started publishing desired cartesian positions"
            
            # Start publishing in a new thread
            self._publish_thread = threading.Thread(target=self.publish_pose)
            self._publish_thread.start()
        else:
            response.success = False
            response.message = "Cartesian Pose has not benn initialised"
            
        return response
    
    def handle_stop_publishing(self, req):
        response = TriggerResponse()
        self._is_publishing = False
        #if self._publish_thread:
            #self._publish_thread.join()
        response.success = True
        response.message = "Maybe False"
        return response




 
    def publish_pose(self):

        while not rospy.is_shutdown() and self._is_publishing:
            # Create PoseStamped message
            pose_stamped = PoseStamped()
            #print(self._cartesian_pose.position.x, self._cartesian_pose.position.y, self._cartesian_pose.position.z)
            # Header
            pose_stamped.header = Header()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "End-effector"

            # Pose
            print(self._cartesian_pose)
            pose_stamped.pose = deepcopy(self._cartesian_pose)
            print(pose_stamped)
            # Publish the message
            self._pub.publish(pose_stamped)
            self._rate.sleep()



if __name__ == '__main__':
    try:
        publisher = CartesianPublisherNode()
    except rospy.ROSInterruptException:
        pass
