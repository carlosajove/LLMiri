import threading
from rosbridgeLLMinterface_class import rosbridgeLLMinterface
from callLLM import PhiMini


def start_interface(interface_instance):
    interface_instance.run_forever()


if __name__=="__main__":
    """
    Input is free (+-), must be pased as type, and be treated in the Callable. The output must be a string
    """
    #MODELS
    phiMini = PhiMini()
    
    # Classes
    cart_stiffness = rosbridgeLLMinterface(phiMini.callSelectStiffness, type="transformer/CallLLM", service_name="phi_mini/callStiffness", topic_name="phi_mini/outputStiffness")
    test = rosbridgeLLMinterface(phiMini.call, type="transformer/CallLLM_long", service_name="phi_mini/call", topic_name="phi_mini/output")
    
    
    # Start each interface in a separate thread
    cart_stiffness_thread = threading.Thread(target=start_interface, args=(cart_stiffness,))
    test_thread = threading.Thread(target=start_interface, args=(test,))
    
    cart_stiffness_thread.start()
    test_thread.start()
    
    # Join threads to the main thread 
    cart_stiffness_thread.join()
    test_thread.join()