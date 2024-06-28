#pragma once

#include <string>
#include <vector>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <ll_control/GetPose.h>


namespace ll_control {

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
        bool getEndEffectorPose(ll_control::GetPose::Request& req, ll_control::GetPose::Response& res);
    private:
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

        ros::ServiceServer srv_get_end_effector_pose;

    };

} // namespace franka_example_controllers
