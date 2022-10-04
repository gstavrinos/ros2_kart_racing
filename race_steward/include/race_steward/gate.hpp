#pragma once
#include "ignition/math/OrientedBox.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"

namespace race_steward {

class Gate {

public:
    Gate(){}
    Gate(const std::string&, const ignition::math::OrientedBoxd&);
    Gate(const boost::shared_ptr<gazebo::physics::Link>&);
    bool operator<(const Gate&) const;
    std::string get_name() const;
    ignition::math::OrientedBoxd get_bounding_box() const;
private:
    std::string name;
    ignition::math::OrientedBoxd bb;

};

}
