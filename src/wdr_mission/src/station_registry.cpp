#include "wdr_mission/station_registry.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace wdr_mission {

namespace {

void parse_into(const YAML::Node& root, std::map<std::string, Station>& stations, std::string& frame_id) {
    if (!root || !root["stations"] || !root["stations"].IsMap()) {
        throw std::runtime_error("stations file has no 'stations' map");
    }
    if (root["frame_id"]) frame_id = root["frame_id"].as<std::string>();
    for (const auto& kv : root["stations"]) {
        Station s;
        s.name = kv.first.as<std::string>();
        const YAML::Node& v = kv.second;
        if (!v["type"] || !v["x"] || !v["y"] || !v["yaw"]) {
            throw std::runtime_error("station '" + s.name + "' needs type, x, y and yaw");
        }
        s.type = v["type"].as<std::string>();
        s.x = v["x"].as<double>();
        s.y = v["y"].as<double>();
        s.yaw = v["yaw"].as<double>();
        stations[s.name] = s;
    }
}

}  // namespace

StationRegistry StationRegistry::load(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("cannot read stations file '" + path + "': " + e.what());
    }
    StationRegistry r;
    parse_into(root, r.stations_, r.frame_id_);
    return r;
}

StationRegistry StationRegistry::parse(const std::string& yaml_text) {
    StationRegistry r;
    parse_into(YAML::Load(yaml_text), r.stations_, r.frame_id_);
    return r;
}

std::optional<Station> StationRegistry::find(const std::string& name) const {
    const auto it = stations_.find(name);
    if (it == stations_.end()) return std::nullopt;
    return it->second;
}

bool StationRegistry::has(const std::string& name, const std::string& type) const {
    const auto s = find(name);
    return s && s->type == type;
}

std::vector<std::string> StationRegistry::names(const std::string& type) const {
    std::vector<std::string> out;
    for (const auto& [name, s] : stations_) {
        if (type.empty() || s.type == type) out.push_back(name);
    }
    return out;
}

std::string StationRegistry::validate_order(const std::string& pickup, const std::string& dropoff) const {
    if (!find(pickup)) return "unknown pickup station '" + pickup + "'";
    if (!find(dropoff)) return "unknown dropoff station '" + dropoff + "'";
    if (!has(pickup, "pickup")) return "'" + pickup + "' is not a pickup station";
    if (!has(dropoff, "dropoff")) return "'" + dropoff + "' is not a dropoff station";
    return "";
}

}  // namespace wdr_mission
