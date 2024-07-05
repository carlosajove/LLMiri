#!/usr/bin/env python


import rospy 

from transformer.srv import CallLLM

rospy.init_node('hello')

rospy.wait_for_service('/transformer/LLM_interface')

service = rospy.ServiceProxy('/transformer/LLM_interface', CallLLM)


a = service('abca')

print('response ', a)