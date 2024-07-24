#!/usr/bin/env python
import rospy
import threading
import re

from transformer.srv import CallLLM, CallLLM_long, GetObjectPoseFromDescription, GetObjectPoseFromDescriptionResponse
from std_msgs.msg import String
from llm_gazebo.msg import NameDescriptionPair, ObjectDescriptionList
from ll_control.srv import GetObjectPose
from transformer.msg import CallLLMsingle, CallLLMlist

class LLM_node():
    def __init__(self):
        rospy.init_node('llm_node')
    
        rospy.loginfo("llm_node initialized")
        # Subscribers
        self.output_message = None
        self._LLM_output_subscriber = rospy.Subscriber('/transformer/phi_mini/output', String, self.LLM_callback)
        self._object_description_subscriber = rospy.Subscriber('llm_context/object_description', ObjectDescriptionList, self.object_description_callback)
        # Services
        self.service = rospy.Service('/transformer/get_object_pose_from_description', GetObjectPoseFromDescription, self.getPoseFromObjectDescription_handle)
        
        #State Triggers
        self._get_pose_from_object_description_send_trigger = False     #True when LLM is called
        self._received_trigger = False #True when subscriber reads a message
        rospy.spin()

        
    def object_description_callback(self, msg):
        self._res = ""
        for object in msg.objects:
            self._res += "name: {}, description: {} |".format(object.name, object.description)
    
    def LLM_callback(self, msg):
        if (self._get_pose_from_object_description_send_trigger): 
            #TODO: solve potential bug if res/res2 not found   

            pattern = r"- OBJECT NAME:\s*(.*)"
            match = re.search(pattern, msg.data)
            
            if match:
                res = match.group(1)
                rospy.loginfo("Extracted OBJECT NAME: {}".format(res))
                self._desired_object_name = res 
                self._received_trigger = True 
            else:
                rospy.loginfo("OBJECT NAME not found in the response.")
                res = None
              

            pattern2 = r"- Explanation:\s*(.*)"
            match2 = re.search(pattern2, msg.data)
            if match:
                res2 = match2.group(1)
                rospy.loginfo("Extracted OBJECT NAME: {}".format(res2))
            else:
                rospy.loginfo("OBJECT NAME not found in the response.")
                res2 = None
            
            self._desired_object_explanation = res2

        
        
    def get2(self, req):
        pass
        
        
    def getPoseFromObjectDescription_handle(self, req):
        # Start a separate thread to call the external service
        INPUT = [CallLLMsingle(role='system', content= "Franka needs to pick up an object. Using the description of the different objects that are on the world, provided below. \
                                                You must tell the user the exact name of the object based on the description provided by the user. The only available objects are the ones provided in the description list.  \
                                                Your object name answer must match one of the presented options. \
                                                The answer must be in the following format: \
                                                           - OBJECT NAME: name (name must match an object in the initial object list of descriptions)   \
                                                           - Explanation: here you will write why you choose that name and give an estimation of your confidence in the choice"),
                 CallLLMsingle(role = 'system', content = "If you think that the user description doesn't match any provided object you must return the OBJECT NAME: None, and the consequent explanaition. Bellow you will find the name and description of all of the objects in the world: " + self._res),
                 CallLLMsingle(role = 'user', content = req.description)]
        input = CallLLMlist()
        input.unit = INPUT
        
        self._get_pose_from_object_description_send_trigger = True
        thread = threading.Thread(target=self.call_external_service, args=(input,))
        thread.start()

        while(not self._received_trigger):
            pass
        #received_trigger = TRUE
        
        #getPose
        rospy.wait_for_service('/read_only_controller/get_object_state')

        get_srv = rospy.ServiceProxy('/read_only_controller/get_object_state', GetObjectPose)
        get_res = get_srv(self._desired_object_name)

        response = GetObjectPoseFromDescriptionResponse()
        response.object_name = self._desired_object_name
        response.pose = get_res.pose 
        response.twist = get_res.twist 
        response.success = get_res.success
        response.explanation = self._desired_object_explanation

        self._received_trigger = False
        self._get_pose_from_object_description_send_trigger = False       
        return response

    def call_external_service(self, input):
        # Call the external service
        rospy.wait_for_service('/transformer/phi_mini/call')
        try:
            callLLM = rospy.ServiceProxy('/transformer/phi_mini/call', CallLLM_long)  
            callLLM(input)  # Make the service call without waiting for response
            rospy.loginfo("LLM call with input '{}' completed.".format(input))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))



if __name__ == "__main__":
    try:
        print("jfalsdkA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!fj")
        a = LLM_node()
    except Exception as e:
            print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
            rospy.logerr('LLM node could not be initialized: {}'.format(e))