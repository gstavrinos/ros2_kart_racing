#include "race_steward/racer.hpp"

namespace race_steward {

    Racer::Racer() : Racer(NULL) {}

    Racer::Racer(const gazebo::physics::ModelPtr m) {
        valid = false;
        laps = 0;
        lap_times.clear();
        curr_lap_time = 0.0;
        start_lap_time = 0.0;
        next_sector_index = 0;
        curr_lap_poses.clear();
        best_lap_poses.clear();
        best_lap = std::numeric_limits<double>::max();
        if (m != NULL) {
            name = m->GetName();
            bb = ignition::math::OrientedBoxd(m->BoundingBox().Size(), m->WorldPose());
        }
        else {
            name = "placeholder";
            bb = ignition::math::OrientedBoxd();
        }
    }

    void Racer::update_data(const gazebo::physics::ModelPtr m, const double t) {
        bb = ignition::math::OrientedBoxd(m->BoundingBox().Size(), m->WorldPose());
        curr_lap_poses.push_back(m->WorldPose());
        if (start_lap_time > 0) {
            curr_lap_time = t - start_lap_time;
        }
    }

    bool Racer::get_valid() const {
        return valid;
    }

    unsigned Racer::get_laps() const {
        return laps;
    }

    double Racer::get_best_lap() const {
        return best_lap;
    }

    ignition::math::OrientedBoxd Racer::get_bounding_box() const {
        return bb;
    }

    std::string Racer::get_name() const {
        return name;
    }

    double Racer::get_curr_lap_time() const {
        return curr_lap_time;
    }

    void Racer::invalidate() {
        valid = false;
    }

    void Racer::set_valid(const bool c) {
        valid = c;
    }

    void Racer::set_laps(const unsigned l) {
        laps = l;
    }

    void Racer::set_best_lap(const double l) {
        best_lap = l;
    }

    void Racer::set_name(const std::string n) {
        name = n;
    }

    unsigned Racer::get_next_sector_index() const {
        return next_sector_index;
    }

    bool Racer::operator==(const std::string & n) const {
        return name == n;
    }

    void Racer::got_through_sector(const unsigned sectors, const double t) {
        if (next_sector_index == 0) {
            // Not first ever pass from the start/end gate
            if (start_lap_time > 0) {
                laps++;
                curr_lap_time = t - start_lap_time;
                lap_times.push_back(curr_lap_time);
                if (curr_lap_time < best_lap) {
                    best_lap = curr_lap_time;
                    best_lap_poses = std::vector(curr_lap_poses);
                }
            }
            start_lap_time = t;
            curr_lap_poses.clear();
        }
        next_sector_index = (next_sector_index + 1 < sectors) * (next_sector_index + 1);
    }

    std::string Racer::to_string() {
        return std::string("\n~~~ [ R A C E R ] ~~~\nName: " + name + "\nLap: " + std::to_string(laps) + "\nCLT: " + std::to_string(curr_lap_time) + "\nPB: " + std::to_string(best_lap) + "\n~~~~~~~~~~~~~~~~~~~~~~\n");
    }

}
