#include "race_steward/track.hpp"

namespace race_steward {

    Track::Track() {
        name = "track";
        finalized = false;
        gates.clear();
        sectors = 0;
    }

    Track::Track(const std::string& n) {
        name = n;
        finalized = false;
        gates.clear();
        sectors = 0;
    }

    void Track::add_gate(const Gate& g) {
        gates.push_back(g);
        sectors = gates.size();
    }

    void Track::finalize() {
        sort(gates.begin(), gates.end());
        finalized = true;
    }

    void Track::set_name(const std::string& s) {
        name = s;
    }

    std::string Track::get_name() const {
        return name;
    }

    bool Track::is_finalized() const {
        return finalized;
    }

    unsigned Track::get_sectors() const {
        return sectors;
    }

    std::string Track::to_string() {
        std::string s = "\n---" + name + "---\n" + "-[ G A T E S ]-\n";
        for (auto g : gates) {
            s += g.get_name() + "\n"; 
        }
        s += "--------------\n";
        return s;
    }

    bool Track::racer_through_gate(const Racer& r) const {
        // This collision check is definitely problematic, since it only checks the center of the racer's bounding box
        return r.get_next_sector_index() < sectors and gates[r.get_next_sector_index()].get_bounding_box().Contains(r.get_bounding_box().Pose().Pos());
    }

}
