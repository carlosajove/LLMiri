#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import re

from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter, GroupState

class alsdfjlj():
    def __init__(self):

        rospy.init_node('interface_LLM_controller')
        self._LLM_subscriber = rospy.Subscriber('/transformer/LLM_interface/output', String, self.LLMoutputCalback)
        rospy.wait_for_service('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters') 

        rospy.spin()
        
    def LLMoutputCalback(self, data):
        pattern = r'cartesian_stiffness:\s*{([^}]*)}'
        
        exp = re.search(pattern, data.data)
        if exp:
            values_str = exp.group(1)
            
            self._cartesian_stiffness = [float(val.strip()) for val in values_str.split(',')]
            print('values', self._cartesian_stiffness)
            
            self.set_controller_stiffness(self._cartesian_stiffness)

        else:
            print("No match found for: ", self._cartesian_stiffness)
       
            
    
    def set_controller_stiffness(self, param):
        rospy.loginfo('Setting Controller Stiffness')
        request = ReconfigureRequest()
        
        request.config.bools = []
        request.config.ints = []
        request.config.strs = []
        request.config.doubles = [
            DoubleParameter(name='translational_stiffness', value=param[0]),
            DoubleParameter(name='rotational_stiffness', value=param[1])
        ]
        request.config.groups = [
            GroupState(name='LLM', state=True, id=0, parent=0)
        ]
        try:
            controller_stiffness_client =  rospy.ServiceProxy('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters', Reconfigure)    
            response = controller_stiffness_client(request)
            print("Parameters updated successfully")
        except rospy.ServiceException as e:
            print("Service call failes %s" % e)
        
        
        
        
            
    #def read
if __name__ == '__main__':
    a = alsdfjlj()

    try:
        b = alsdfjlj()
    except:
        print("ERROR LLM to cartesian")