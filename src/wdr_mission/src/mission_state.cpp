#include "wdr_mission/mission_state.hpp"

#include "wdr_interfaces/action/deliver_order.hpp"

namespace wdr_mission {

using Result = wdr_interfaces::action::DeliverOrder::Result;

bool is_phase_state(const std::string& s) {
    return s != state::kNavRetry && s != state::kToException && s != state::kWaitingForNav2;
}

uint16_t error_code_for_phase(const std::string& phase) {
    if (phase == state::kGoingToCharge || phase == state::kCharging || phase == state::kUndocking ||
        phase == state::kCheckingBattery) {
        return Result::CHARGING_FAILED;
    }
    if (phase == state::kToPickup) return Result::PICKUP_UNREACHABLE;
    if (phase == state::kWaitingForLoad) return Result::LOAD_TIMEOUT;
    if (phase == state::kToDropoff) return Result::DROPOFF_UNREACHABLE;
    if (phase == state::kWaitingForUnload) return Result::UNLOAD_TIMEOUT;
    return Result::UNKNOWN;
}

}  // namespace wdr_mission
