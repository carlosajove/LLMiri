# LLMiri

Framework for Using LLMs to Control Franka's Robotic Arm

## Overview
LLMiri is a framework designed to leverage Large Language Models (LLMs) for controlling Franka's robotic arm. This project integrates ROS-Melodic and franka_ros to enable interaction with the low-level controller of the robotic arm using natural language commands.

## Features
- **Integration with ROS-Melodic**: Utilizes the Robot Operating System (ROS) Melodic for communication and control.
- **franka_ros Compatibility**: Interfaces with Franka Emika's robot arm using the franka_ros package.
- **Natural Language Processing**: Employs LLMs to interpret and execute commands for robotic control.
- **Modular Design**: Easily extendable to incorporate additional functionalities and improvements.

### Prerequisites

- Ubuntu 18.04
- ROS-Melodic installed
- franka_ros package installed and configured ([installation guide](https://frankaemika.github.io/docs/installation_linux.html))
- Gazebo installed and configured
- Anaconda or Miniconda istalled [install Conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html)
- Python 2.7, Python 3.8+
- requirements.txt and versions may not be the correct ones (I don't have access to the computer right now)

### Setup

1. Create Catkin workspace in a directory of your choice:
   ```sh
   cd /path/to/desired/folder
   mkdir -p catkin_ws/src
   cd catkin_ws
   source /opt/ros/melodic/setup.sh
   catkin_init_workspace src
   ```
   
2. Clone the repository:
    ```sh
    cd src
    git clone https://github.com/carlosajove/LLMiri.git
    cd LLMiri
    ```

3. Install the required dependencies:
    ```sh
    pip install -r requirements.txt
    ```

4. Build the ROS workspace:
    ```sh
    cd ../../catkin_ws
    catkin_make
    source devel/setup.bash
    ```


5. Create the conda environment with python 3:
    ```sh
    conda create -n py3 python=3.12
    conda activate py3
    pip install torch transformers webocket-client
    conda deactivate 
    ```


### Usage example

1. Start the ROS nodes:
   ```sh
   roscore
   roslaunch llm_gazebo custom_panda_launch.launch
   ```

2. Send commands to the robotic arms using natural language via ROS services:
   ```sh
   rosservice call /hl_control/pick_object_from_description "pick the smallest object"


## Module Overview

Each one of the following are implemented as ROS packages, and can be used via their respective topics and services. 

### 1. Control

The `control` module handles the low-level control with Franka's robotic arm. It manages the interface with the `franka_ros` package, sending and receiving commands to control the robot's movement and behavior.

[More about the Control module](control/README.md)

### 2. HL Control

The `hl_control` module is responsible for interpreting high-level commands, typically derived from natural language. This module bridges the gap between the LLM responses and the lower-level control mechanisms.

[More about the HL Control module](hl_control/README.md)

### 3. LLM Gazebo

`LLM_gazebo` serves as an interface for gazebo.

[More about the LLM Gazebo module](LLM_gazebo/README.md)

### 4. Transformer

The `transformer` module leverages seves as an interface with Large Language Models (LLMs). 

[More about the Transformer module](transformer/README.md)