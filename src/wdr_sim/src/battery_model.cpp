#include "wdr_sim/battery_model.hpp"

#include <algorithm>
#include <cmath>

namespace wdr_sim {

BatteryModel::BatteryModel(const BatteryParams& params, double initial_soc) : params_(params), soc_(0.0) {
    set_soc(initial_soc);
}

void BatteryModel::set_soc(double soc) { soc_ = std::clamp(soc, 0.0, 1.0); }

double BatteryModel::voltage() const {
    return params_.empty_voltage + (params_.full_voltage - params_.empty_voltage) * soc_;
}

void BatteryModel::update(double dt_s, double speed_mps, bool on_charger) {
    if (dt_s <= 0.0) return;
    const double load_w = params_.idle_power_w + params_.drive_power_w_per_mps * std::abs(speed_mps);
    double net_w = -load_w * params_.drain_scale;
    // The charger covers the robot's own load and charges with what is left. A full battery
    // stops drawing charge current, as a real charger's constant-voltage phase tapers off.
    charging_ = on_charger && soc_ < 1.0;
    if (on_charger) net_w = soc_ < 1.0 ? params_.charge_power_w * params_.charge_scale : 0.0;

    const double capacity_j = params_.capacity_wh * 3600.0;
    set_soc(soc_ + net_w * dt_s / capacity_j);
    current_a_ = net_w / voltage();
}

}  // namespace wdr_sim
