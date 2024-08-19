### hl_control

`LLM_to_controller.py` publishes services to be used by the user of the arm. Currently three services are implemented `/hl_control/move_to_object_from_description`, `/hl_control/pick_object_from_description`, `/hl_control/set_cartesian_stiffness_from_description`. LLM is used to give a command from a description "string" written by the user. 

To implement new funcitonalities (new services), a new service should also be implemended in `transformer/src/rosmelodic/llm_node.py` to translate the LLM output from a string format to a format that can be interpreted by the controller.
