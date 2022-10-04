#include "race_steward/race_steward.hpp"

namespace race_steward {


RaceStewardPlugin::RaceStewardPlugin() {}

void RaceStewardPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr sdf) {
    node_ = gazebo_ros::Node::Get(sdf);
    rclcpp::Node::SharedPtr ros2_node_ = std::dynamic_pointer_cast<rclcpp::Node>(node_);
    publisher_ = ros2_node_->create_publisher<race_steward_msgs::msg::RaceStewardLiveInfo>("race_steward/live_info", 10);
    world_ = _world;
    world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&RaceStewardPlugin::OnUpdate, this, std::placeholders::_1));
    auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
    track_model_name = sdf->Get<std::string>("track_model_name", "kartland").first;
    gate_prefix = sdf->Get<std::string>("gate_prefix", "gate_").first;
    racer_prefix = sdf->Get<std::string>("racer_prefix", "kart_").first;
    auto standing_start = sdf->Get<bool>("standing_start", false).first;
    auto quiet = sdf->Get<bool>("quiet", false).first;
    gazebo::common::Console::SetQuiet(quiet);
    update_rate = update_rate <= 0 ? 100 : update_rate;
    update_period_ = 1.0 / update_rate;
    last_update_time_ = _world->SimTime();
    gzmsg << std::string(10, '-') << std::endl;
    gzmsg << "Starting the race steward plugin with the following configuration:" << std::endl;
    gzmsg << "update_rate: " << update_rate  << std::endl;
    gzmsg << "track_model_name" << track_model_name << std::endl;
    gzmsg << "gate_prefix: " << gate_prefix<< std::endl;
    gzmsg << "racer_prefix: " << racer_prefix << std::endl;
    gzmsg << "standing_start: " << standing_start << std::endl;
    gzmsg << std::string(10, '=') << std::endl;
}

void RaceStewardPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {
    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
    if (seconds_since_last_update < update_period_) {
        return;
    }
    gzmsg << "===" << std::endl;
    for (const auto & model : world_->Models()) {
        // If not already done so, initialize the track
        if (not track.is_finalized() and model->GetName() == track_model_name) {
            track.set_name(track_model_name);
            for (const auto & link : model->GetLinks()) {
                if (link->GetName().rfind(gate_prefix,0) == 0) {
                    track.add_gate(Gate(link));
                }
            }
            track.finalize();
            gzmsg << track.to_string();
        }
        // Detect racers
        if (model->GetName().rfind(racer_prefix,0) == 0) {
            auto ri = find(racers.begin(), racers.end(), model->GetName());
            // New racer
            if (ri == racers.end()) {
                racers.push_back(Racer(model));
                ri = racers.end()-1;
            }
            // Old racer, update data
            else {
                ri->update_data(model, _info.simTime.Double());
            }
            // Make sure we have all track sectors 
            // before starting recording lap times
            if (track.is_finalized() and track.racer_through_gate(*ri)) {
                ri->got_through_sector(track.get_sectors(), _info.simTime.Double());
            }
        }
    }
    race_steward_msgs::msg::RaceStewardLiveInfo rsli;
    rsli.track_name = track_model_name;
    for (auto & racer : racers) {
        race_steward_msgs::msg::RaceStewardRacer rsr;
        rsr.name = racer.get_name();
        rsr.lap = racer.get_laps()+1;
        rsr.current_lap_time = racer.get_curr_lap_time();
        rsr.personal_best = racer.get_best_lap();
        rsli.racers.push_back(rsr);
        gzmsg << racer.to_string();
    }
    publisher_->publish(rsli);
    gzmsg << "---" << std::endl;
    last_update_time_ = _info.simTime;
}

GZ_REGISTER_WORLD_PLUGIN(RaceStewardPlugin)

}
