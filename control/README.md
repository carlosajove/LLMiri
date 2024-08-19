### control


`src/EndEffTRajectoryManager.py` creates services and interacts with `franka_ros` in order to move the end-effector of Franka's robotic arm. Uses the dmp trajectories from `my_dmpbbo` to set a target for the cartesian impedance controller from `franka_ros`.

`src/GripperManager.py` creates services to manage the gripper via `franka_ros`.

`read_only_controller_Node.cpp` is used to get information about the robot state: for example end-effector pose.