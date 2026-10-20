// Simple energy model of the robot's battery, kept free of ROS so it can be unit tested.
#pragma once

namespace wdr_sim {

struct BatteryParams {
    double capacity_wh = 26.0;            // Create 3 battery: 26 Wh, 4S Li-ion
    double idle_power_w = 6.0;            // computer, lidar, base electronics
    double drive_power_w_per_mps = 25.0;  // extra draw per m/s of speed
    double charge_power_w = 30.0;         // dock charging power
    double drain_scale = 1.0;             // >1 makes the battery run down faster (demos and tests)
    double charge_scale = 1.0;            // >1 makes charging faster
    double trickle_current_a = 0.6;       // float charge on a full battery while on the charger
    double empty_voltage = 12.0;
    double full_voltage = 16.8;
};

class BatteryModel {
public:
    BatteryModel(const BatteryParams& params, double initial_soc);

    // Advances the model by dt seconds. speed_mps is the robot's absolute linear speed.
    void update(double dt_s, double speed_mps, bool on_charger);

    void set_soc(double soc);
    double soc() const { return soc_; }  // state of charge, 0..1
    bool charging() const { return charging_; }
    double voltage() const;
    // Battery current in amperes: positive while charging, negative while discharging
    // (sensor_msgs/BatteryState convention). A full battery on the charger still shows the
    // small trickle current: opennav_docking's SimpleChargingDock (use_battery_status) reads
    // "current above charging_threshold" as "the robot is on the contacts".
    double current_a() const { return current_a_; }

private:
    BatteryParams params_;
    double soc_;
    bool charging_ = false;
    double current_a_ = 0.0;
};

}  // namespace wdr_sim
