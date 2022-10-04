#pragma once
#include "gazebo/physics/Model.hh"

namespace race_steward {

class Racer {

public:
    Racer();
    Racer(const gazebo::physics::ModelPtr);

    bool get_valid() const;
    unsigned get_laps() const;
    double get_best_lap() const;
    std::string get_name() const;
    double get_curr_lap_time() const;
    unsigned get_next_sector_index() const;
    std::vector<double> get_lap_times() const;
    ignition::math::OrientedBoxd get_bounding_box() const;
    std::vector<ignition::math::Pose3d> get_best_lap_poses() const;

    void invalidate();
    void set_valid(const bool);
    void set_laps(const unsigned);
    void set_best_lap(const double);
    void set_name(const std::string);

    std::string to_string();
    bool operator==(const std::string&) const;
    void update_data(const gazebo::physics::ModelPtr, const double);
    void got_through_sector(const unsigned, const double);

private:
    bool valid;
    std::string name;
    std::vector<double> lap_times;
    ignition::math::OrientedBoxd bb;
    unsigned laps, next_sector_index;
    double best_lap, curr_lap_time, start_lap_time;
    std::vector<ignition::math::Pose3d> best_lap_poses;
    std::vector<ignition::math::Pose3d> curr_lap_poses;
};

}
