// Named stations (pick-up shelves, drop-off zones, docks) loaded from the generated
// stations.yaml, so orders can say "shelf_a1 -> pack_2" instead of coordinates.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace wdr_mission {

struct Station {
    std::string name;
    std::string type;  // pickup | dropoff | dock | exception
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class StationRegistry {
public:
    StationRegistry() = default;

    // Throws std::runtime_error on a missing file or malformed entries.
    static StationRegistry load(const std::string& path);
    // Same format, from a YAML string (used by the tests).
    static StationRegistry parse(const std::string& yaml_text);

    std::optional<Station> find(const std::string& name) const;
    bool has(const std::string& name, const std::string& type) const;
    std::vector<std::string> names(const std::string& type = "") const;
    const std::string& frame_id() const { return frame_id_; }

    // Why an order between these stations cannot be accepted, or an empty string if it can.
    std::string validate_order(const std::string& pickup, const std::string& dropoff) const;

private:
    std::string frame_id_ = "map";
    std::map<std::string, Station> stations_;
};

}  // namespace wdr_mission
