#include <string>
#include <vector>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <ros/ros.h>
#include <Eigen/Dense>


#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>
#include <control/SetPose.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <control/read_only_controller_Node.h>

namespace control {

  bool ReadOnlyController::init(hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& root_node_handle,
    ros::NodeHandle& controller_node_handle) {
    std::cout << "[CAJ] read_only_controller INIT" << std::endl;
    //MODEL INTERFACE AND HANDLE
    std::string arm_id;
    if (!root_node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
      return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
      ROS_ERROR_STREAM(
        "READ ONLY CONTROLLER: Error getting model interface from hardware");
      return false;
    }
    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
        "READ ONLY CONTROLLER: Exception getting model handle from interface: "
        << ex.what());
      return false;
    }

    //STATE INTERFACE AND HANDLE
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM(
        "READ ONLY CONTROLLER: Error getting state interface from hardware");
      return false;
    }
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
        "READ ONLY CONTROLLER: Exception getting state handle from interface: "
        << ex.what());
      return false;
    }

    //Subscibers
    model_states_sub_ = root_node_handle.subscribe("/gazebo/model_states", 1, &ReadOnlyController::modelStatesCallback, this);

    //Services
    get_end_effector_pose_srv_ =
      root_node_handle.advertiseService("read_only_controller/get_end_effector_pose", &ReadOnlyController::getEndEffectorPose, this);

    get_object_model_srv_ =
      root_node_handle.advertiseService("read_only_controller/get_object_state", &ReadOnlyController::getObjectState, this);

    return true;
  }
  void ReadOnlyController::starting(const ros::Time& /*time*/) {
    std::cout << "[CAJ] starting read_only_controller" << std::endl;

  }

  void ReadOnlyController::update(const ros::Time& time, const ros::Duration& period) {
    // This is the realtime loop of your controller. Since its read-only, you can simply do nothing
    // here. Note though, that you want to avoid blocking calls here, since this can trigger
    // communication_constraints_reflex if the update loop is too slow
  }


  bool ReadOnlyController::getEndEffectorPose(control::GetPose::Request& req, control::GetPose::Response& res) {
    Eigen::Affine3d end_eff_isom(Eigen::Matrix4d::Map(model_handle_->getPose(franka::Frame::kEndEffector).data()));
    Eigen::Vector3d end_effector_pos(end_eff_isom.translation());
    Eigen::Quaterniond end_effector_quat(end_eff_isom.rotation());


    res.position_x = end_effector_pos(0);
    res.position_y = end_effector_pos(1);
    res.position_z = end_effector_pos(2);
    res.orientation_x = end_effector_quat.x();
    res.orientation_y = end_effector_quat.y();
    res.orientation_z = end_effector_quat.z();
    res.orientation_w = end_effector_quat.w();
    res.success = true;

    return true;
  }


  //May change this, does't seem fast
  void ReadOnlyController::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // Clear the map before updating with new data
    model_states_ = *msg;
  }

  bool ReadOnlyController::getObjectState(control::GetObjectPose::Request& req, control::GetObjectPose::Response& res) {
    // Search for the object name in the stored model states
    auto it = std::find(model_states_.name.begin(), model_states_.name.end(), req.object_name);
    if (it != model_states_.name.end()) {
      size_t index = std::distance(model_states_.name.begin(), it);
      res.pose = model_states_.pose[index];
      res.twist = model_states_.twist[index];
      res.success = true;
    }
    else res.success = false;

    return true;
  }



} // namespace franka_example_controllers
PLUGINLIB_EXPORT_CLASS(control::ReadOnlyController, controller_interface::ControllerBase)