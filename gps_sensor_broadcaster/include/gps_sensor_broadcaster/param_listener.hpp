// param_listener.hpp
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

struct Params
{
    std::vector<std::string> sensors;
    std::vector<std::string> interfaces;
    bool use_local_topics{false};
    std::string frame_id;
    struct MapInterface
    {
        std::string position;
    } map_interface_to_joint_state;
};

class ParamListener
{
public:
    explicit ParamListener(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
        : node_(node) {}

    Params get_params()
    {
        Params p;
        node_->get_parameter("sensors", p.sensors);
        node_->get_parameter("interfaces", p.interfaces);
        node_->get_parameter("use_local_topics", p.use_local_topics);
        node_->get_parameter("frame_id", p.frame_id);
        node_->get_parameter("map_interface_to_joint_state/position",
                             p.map_interface_to_joint_state.position);
        return p;
    }

private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};
