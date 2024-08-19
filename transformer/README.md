### transformer

Interfaces with LLM.

## rosbridge 

Since ROS Melodic doesn't support Python 3, we use `rosbridge` which allows communication with ROS via websockets.

Currently we use 'microsoft/Phi-3-mini-128k-instruct' from huggingface. To use another LLM a new class can be created in `callLLM.py`. And new services can also be created in `create_rosbridge_srv.py` with the new LLM, by following the examples. 

In the current interface the LLM be directly called with the service `phi_mini/call` and the LLM answer will be published on the topic `phi_mini/output`. In both cases with a string format. 

## rosmelodic

Calls `rosbridge` but is used handle more complex tasks. Normally is called form the `hl_control` module. It gives more context to the LLM about the task to be completed appart form the user input and sets the output string format. It is allso used to transform string format LLM output to numeric output for the parameters to be used by `hl_control`.