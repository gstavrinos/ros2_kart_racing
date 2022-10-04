#pragma once
#include "ignition/math/OrientedBox.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/common/Time.hh"
#include "gazebo_ros/node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "race_steward/track.hpp"
#include "race_steward/racer.hpp"
#include "race_steward_msgs/msg/race_steward_racer.hpp"
#include "race_steward_msgs/msg/race_steward_live_info.hpp"

namespace race_steward {

class RaceStewardPlugin : public gazebo::WorldPlugin {

public:
    RaceStewardPlugin();
    // virtual ~RaceStewardPlugin();
    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:
    void OnUpdate(const gazebo::common::UpdateInfo &_info);

    Track track;
    double update_period_;
    std::vector<Racer> racers;
    gazebo::physics::WorldPtr world_;
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr ros2_node_;
    gazebo::common::Time last_update_time_;
    gazebo::event::ConnectionPtr world_update_event_;
    std::string track_model_name, gate_prefix, racer_prefix;
    rclcpp::Publisher<race_steward_msgs::msg::RaceStewardLiveInfo>::SharedPtr publisher_;
};

}
