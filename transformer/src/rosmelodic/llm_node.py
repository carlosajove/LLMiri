#!/usr/bin/env python
import rospy
import threading

from transformer.srv import (CallLLM, CallLLM_long, 
                             GetObjectPoseFromDescription, GetObjectPoseFromDescriptionResponse, 
                             GetGripperForceFromDescription, GetGripperForceFromDescriptionResponse,
                             GetCartesianStiffnessFromDescription, GetCartesianStiffnessFromDescriptionResponse)
from std_msgs.msg import String
from llm_gazebo.msg import NameDescriptionPair, ObjectDescriptionList
from control.srv import GetObjectPose
from transformer.msg import CallLLMsingle, CallLLMlist




import re    
     
def pattern_match(pattern, data):     
    match = re.search(pattern, data)
    
    if match:
        res = match.group(1)
        rospy.loginfo("Extracted : {}".format(res))
        succes = True 
    else:
        rospy.loginfo(pattern + " not found in the response.")
        res = None
        succes
    
    return res, succes
              




class LLM_node():
    def __init__(self):
        rospy.init_node('llm_node')
    
        rospy.loginfo("llm_node initialized")
        # Subscribers
        self.output_message = None
        self._LLM_output_subscriber = rospy.Subscriber('/transformer/phi_mini/output', String, self.LLM_callback)
        self._object_description_subscriber = rospy.Subscriber('llm_context/object_description', ObjectDescriptionList, self.object_description_callback)
        # Services
        self._obj_pose_srv = rospy.Service('/transformer/get_object_pose_from_description', GetObjectPoseFromDescription, self.getPoseFromObjectDescription_handle)
        self._get_pose_from_object_description_send_trigger = False     #True when LLM is called
        self._get_pose_from_object_description_received_trigger = False #True when subscriber reads a message
        
        self._gripper_force_srv = rospy.Service('/transformer/get_gripper_force_from_description', GetGripperForceFromDescription, self.getGripperForceFromDescription_handle)
        self._gripper_force_send_trigger = False
        self._gripper_force_received_trigger = False
        
        self._set_cartesian_stiffness_srv = rospy.Service('transformer/set_cartessian_stiffness_from_description', GetCartesianStiffnessFromDescription, self.getCartesianStiffnessFromDescription_handle)
        self._cartesian_stiffness_send_trigger = False
        self._cartesian_stiffness_received_trigger = False
        rospy.spin()


    def call_external_service(self, input):
        # Call the external service
        rospy.wait_for_service('/transformer/phi_mini/call')
        try:
            callLLM = rospy.ServiceProxy('/transformer/phi_mini/call', CallLLM_long)  
            callLLM(input)  # Make the service call without waiting for response
            rospy.loginfo("LLM call with input '{}' completed.".format(input))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

        
    def object_description_callback(self, msg):
        self._res = ""
        for object in msg.objects:
            self._res += "name: {}, description: {} |".format(object.name, object.description)
    
    def LLM_callback(self, msg):
        #TODO: only one send trigger can be true at the same time
        #TODO: optionally, in rosbridge could publish on different topics. This can be solved by using python 3 with ros noetic, and directly using services without topics
        #TODO: error handling
        if (self._get_pose_from_object_description_send_trigger): 
            #TODO: solve potential bug if res/res2 not found   

            pattern = r"- OBJECT NAME:\s*(.*)"
            self._desired_object_name, a = pattern_match(pattern, msg.data)

            pattern2 = r"- Explanation:\s*(.*)"
            self._desired_object_explanation, self._get_pose_from_object_description_received_trigger = pattern_match(pattern2, msg.data)
            
        elif (self._gripper_force_send_trigger):
            pattern = r"- GRIPPER FORCE:\s*(.*)"
            pattern1 = r"- GRIPPER WIDTH:\s*(.*)"
            pattern2 = r"- Explanation:\s*(.*)"
            
            self._gripper_force, a = pattern_match(pattern, msg.data)
            self._gripper_width,a = pattern_match(pattern1, msg.data)
            self._gripper_explanation, self._gripper_force_received_trigger = pattern_match(pattern2, msg.data)
        
        elif (self._cartesian_stiffness_send_trigger):
            pattern = r"- translational_stiffness:\s*(\d+\.?\d*)"
            pattern1 = r"- rotational_stiffness:\s*(\d+\.?\d*)"
            pattern2 = r"- Explanation:\s*(.*)"
            
            self._translational_stiffness, a = pattern_match(pattern, msg.data)
            self._rotational_stiffness, a = pattern_match(pattern1, msg.data)
            self._stiffness_explanation, self._cartesian_stiffness_received_trigger = pattern_match(pattern2, msg.data)

            
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

        while(not self._get_pose_from_object_description_received_trigger):
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

        self._get_pose_from_object_description_received_trigger = False
        self._get_pose_from_object_description_send_trigger = False       
        return response


    def getGripperForceFromDescription_handle(self, req):
        INPUT = [CallLLMsingle(role='system', content= "Franka needs to pick up an object. Using the description of the different objects that are on the world, provided below, you are going to tell me what are the parameters i should use. \
                                        You must tell what is the correct gripper force that must be applied to pick up the desired object.  \
                                        The answer must be in the following format: \
                                                - GRIPPER FORCE: value (numeric in Newtons) \
                                                - GRIPPER WIDTH: value (numeric in meters) \
                                                - Explanation: here you will write why you choose those values and give an estimation of your confidence in the choice in percentage"),
        CallLLMsingle(role = 'system', content = "an example of output is: - GRIPPER FORCE: 50 \n - GRIPPER WIDTH: 0.05 \n - Explanation: ..... This is only to give an idea of the format the values do not matter in this example."),
        CallLLMsingle(role = 'system', content = "Bellow you will find the name and description of all of the objects in the world: " + self._res),
        #CallLLMsingle(role = 'user', content = 'The stone is around 3 cm wide and 50 g heavy'),
        #CallLLMsingle(role = 'assistant', content = ' - GRIPPER FORCE: 50 \n - GRIPPER WIDTH: 0.03 \n - Explanation: Since gravity is around 10 ms^-2, gripper force should be of 50 N for an object of 50 grams. The stone has a width of 3cm therefore the gripper width must be of 0.03 meters. My confidence is high 80% since i have good information.'),
        CallLLMsingle(role = 'user', content = req.description)]

        input = CallLLMlist()
        input.unit = INPUT
        
        self._gripper_force_send_trigger = True
        thread = threading.Thread(target=self.call_external_service, args=(input,))
        thread.start()
        
        while(not self._gripper_force_received_trigger):
            pass

        response = GetGripperForceFromDescriptionResponse()
        response.force = float(self._gripper_force)
        response.width = float(self._gripper_width)
        response.explanation = self._gripper_explanation
        response.success = True #TODO: change, error handling
        
        return response

    def getCartesianStiffnessFromDescription_handle(self, req):
        INPUT = [CallLLMsingle(role='system', content="The robot comes with a cartesian-impedance controller that takes in a cartesian position and orientation. \
                                        The behaviour of the controller can be tuned via parameters. \
                                        The cartesian stiffness parameter defines how resistant the end-effector is to displacement when a force or external disturbance is applied to it. It is a 6-dimensional vector (kx, ky, kz, kroll, kpitch, kyaw). \
                                        kx = ky = kz = translational_stiffness are the stiffness in the translational directions, kroll = kpitch = kyaw = rotational_stiffness are the orientation stiffness. \
                                            \
                                        Your job is to tell what parameters to use in the controller depending on the context.\
                                        \
                                        Your answer will consist of the following parts: \
                                        Explanation: in this part you must explain in words the choices you make. \
                                        Parameters: In this part you must write explicitly the paramter values that enable you to follow the desired behaviour. \
                                        The answer must be in the following format:  \
                                            - translational_stiffness: value \
                                            - rotational_stiffness: value \
                                            - Explanation: ... \
                                        Where translational_stiffness, rotational_stiffness are values between 0.0 and 1.0, in float format."),
                 CallLLMsingle(role = 'user', content = "I'm trying to move a rock."),
                 CallLLMsingle(role = 'assistant', content = " - translational_stiffness: 0.8 \
                                                               - rotational_stiffness: 0.5 \
                                                               - Explanation: Rocks are heavy objects therefore you should use a high stiffness. "),
                 CallLLMsingle(role = 'user', content = req.description)]
        
        input = CallLLMlist()
        input.unit = INPUT
        
        self._cartesian_stiffness_send_trigger = True
        thread = threading.Thread(target=self.call_external_service, args=(input,))
        thread.start()
        
        while(not self._cartesian_stiffness_received_trigger):
            pass
        
        response = GetCartesianStiffnessFromDescriptionResponse()
        response.translational_stiffness = float(self._translational_stiffness)
        response.rotational_stiffness = float(self._rotational_stiffness)
        response.explanation = self._stiffness_explanation
        response.success = True  
        
        return response
        
        

if __name__ == "__main__":
    try:
        a = LLM_node()
    except Exception as e:
            rospy.logerr('LLM node could not be initialized: {}'.format(e))