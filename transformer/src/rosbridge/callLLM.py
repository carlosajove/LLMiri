import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline 



class PhiMini():
    def __init__(self):
        torch.random.manual_seed(0) 
        self._model = AutoModelForCausalLM.from_pretrained( 
            "microsoft/Phi-3-mini-128k-instruct",  
            device_map="cuda",  
            torch_dtype="auto",  
            trust_remote_code=True,
        ) 
        self._tokenizer = AutoTokenizer.from_pretrained("microsoft/Phi-3-mini-128k-instruct") 
        self._pipe = pipeline( 
            "text-generation", 
            model=self._model, 
            tokenizer=self._tokenizer, 
        ) 

        self._generation_args = { 
            "max_new_tokens": 500, 
            "return_full_text": False, 
            "temperature": 0.01, 
            "do_sample": True,
        } 
        self._context = None
    
    
    def callSelectStiffness(self, input, save_context = False): #TODO: Implement save_context feature: save the text and output in a list? chat history
        messages = [ 
            {"role": "system", "content": "You are a helpful robotics controls engineer. \
                                        You have a robot arm which is the Franka Emika Panda robot arm, a single robot arm with 7 degrees of freedom. \
                                        The robot has a parallel-jaw gripper equipped with two small finger pads. The robot comes with a cartesian-impedance controller that takes in a cartesian position and orientation. \
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
                                            - Explanation: ... \
                                            - Parameters: cartesian_stiffness: {translational_stiffness, rotational_stiffness} \
                                        Where translational_stiffness, rotational_stiffness are values between 0 and 1. \
                                            "}, 
            {"role": "user", "content": "I'm trying to move a rock."}, 
            {"role": "assistant", "content": "Explanation: Rocks are heavy objects therefore you should use a high stiffness. \
                                            Parameters: cartesian_stiffness: {0.8, 0.5}"}, 
            {"role": "user", "content": "Now I'm trying to move a " + input}, 
        ] 

        output = self._pipe(messages, **self._generation_args) 
        print('output', output)
        return output[0]['generated_text']
    
    def call(self, input, save_context = False):
        print("[INFO] PhiMini call")
        print("[input]", input['unit'])
        messages = [
            {"role": "system", "content": "You are a helpful robotics controls engineer. \
                                        You have a robot arm which is the Franka Emika Panda robot arm, a single robot arm with 7 degrees of freedom. \
                                        The robot has a parallel-jaw gripper equipped with two small finger pads. The robot comes with a cartesian-impedance controller that takes in a cartesian position and orientation. \
                                        The behaviour of the controller can be tuned via parameters. Your role is to answer all of the questions that the user does."},
             *input['unit']]
        
        print('\n \n \n', messages, '\n \n \n')
        
        
        
        
        output = self._pipe(messages, **self._generation_args) 
        print('output', output)
        return output[0]['generated_text']
