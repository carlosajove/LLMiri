### LLM_gazebo

Primary use is to interact with gazebo simulation. Launches the simulation with `custom_panda.launch`, which spaws the robot and objects and runs the principal nodes.

Since the LLM used doesn't have vision input, the LLM needs written context about the different objects in the space. This is provided by the `object_description_publisher.py`