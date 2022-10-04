#include "race_steward/gate.hpp"

namespace race_steward {
    Gate::Gate(const std::string& n, const ignition::math::OrientedBoxd& b) {
        name = n;
        bb = b;
    }

    Gate::Gate(const boost::shared_ptr<gazebo::physics::Link>& link) {
        name = link->GetName();
        bb = ignition::math::OrientedBoxd(link->BoundingBox().Size(), link->WorldPose());
    }

    std::string Gate::get_name() const {
        return name;
    }

    ignition::math::OrientedBoxd Gate::get_bounding_box() const {
        return bb;
    }

    bool Gate::operator<(const Gate& g) const {
        return this->name < g.get_name();
    }
}
