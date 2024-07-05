#!/usr/bin/env python

import rospy
import subprocess
import json

from std_srvs.srv import Trigger, TriggerResponse

class llmInterfaceNode():
    def __init__(self):
        rospy.init_node('llm_interface')
        
        self._call_llm = rospy.Service('/transformer/llm/interface', Trigger, self._handle_LLMservice)
        
        self._transformer_script = '/home/carribalzaga/Desktop/franka/catkin_ws/src/LLMiri/transformer/test_models/microsoft_phi_128k.py'
        rospy.loginfo("LLM Interface Service Ready!")
        rospy.spin()
    
    def _handle_LLMservice(self, req):
        # Define the command to activate the virtual environment and run the Python 3 script
        command = [
            "bash", "-c", 
            "source py3env/bin/activate", "&&", "python3 " + self._transformer_script + "\"{}\"".format(req)
        ]
        try:
            result = subprocess.check_output(command) 
            result = json.loads(result.decode("utf-8"))  # Decode the result
            rospy.loginfo(f"LLM generated: {result['generated_text']}")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error running LLM: {e}")
            result = {"generated_text": "Error running LLM"}
        
        return TriggerResponse(success=True, message=result['generated_text'])
    
    
if __name__ == '__main__':
    
    a = llmInterfaceNode()