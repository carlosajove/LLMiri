#pragma once

#include <string>
#include <vector>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <control/GetPose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <control/GetObjectPose.h>


namespace control {

    class ReadOnlyController
        : public controller_interface::MultiInterfaceController<franka_hw::FrankaStateInterface,
        franka_hw::FrankaModelInterface> {

    public:
        bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_node_handle,
            ros::NodeHandle& controller_node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

        //Services
    private:
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;


        ros::Subscriber model_states_sub_;
        ros::ServiceServer get_object_model_srv_;
        gazebo_msgs::ModelStates model_states_;
        void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        bool getObjectState(control::GetObjectPose::Request& req, control::GetObjectPose::Response& res);




        ros::ServiceServer get_end_effector_pose_srv_;
        bool getEndEffectorPose(control::GetPose::Request& req, control::GetPose::Response& res);


    };

} // namespace franka_example_controllers
