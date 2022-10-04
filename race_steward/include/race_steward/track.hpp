#pragma once
#include <iostream>
#include "race_steward/gate.hpp"
#include "race_steward/racer.hpp"

namespace race_steward {

class Track {

public:
    Track();
    Track(const std::string&);

    void finalize();
    std::string to_string();
    bool is_finalized() const;
    void add_gate(const Gate&);
    std::string get_name() const;
    unsigned get_sectors() const;
    void set_name(const std::string&);
    bool racer_through_gate(const Racer&) const;

private:
    bool finalized;
    std::string name;
    unsigned sectors;
    std::vector<Gate> gates;

};

}
