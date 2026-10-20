// Mission states reported by the behavior tree and how a failure in each maps to the
// DeliverOrder result code.
#pragma once

#include <cstdint>
#include <string>

namespace wdr_mission {

namespace state {
inline constexpr const char* kQueued = "QUEUED";
inline constexpr const char* kCheckingBattery = "CHECKING_BATTERY";
inline constexpr const char* kGoingToCharge = "GOING_TO_CHARGE";
inline constexpr const char* kCharging = "CHARGING";
inline constexpr const char* kUndocking = "UNDOCKING";
inline constexpr const char* kToPickup = "TO_PICKUP";
inline constexpr const char* kWaitingForLoad = "WAITING_FOR_LOAD";
inline constexpr const char* kToDropoff = "TO_DROPOFF";
inline constexpr const char* kWaitingForUnload = "WAITING_FOR_UNLOAD";
inline constexpr const char* kNavRetry = "NAV_RETRY";
inline constexpr const char* kToException = "TO_EXCEPTION_AREA";
inline constexpr const char* kDone = "DONE";
inline constexpr const char* kReturningToDock = "RETURNING_TO_DOCK";
}  // namespace state

// States that only describe *how* the robot is doing something (a retry, the trip to the
// exception area after a failure) do not change the mission phase used for error reporting.
bool is_phase_state(const std::string& s);

// DeliverOrder result error code for an order that failed while in `phase`.
uint16_t error_code_for_phase(const std::string& phase);

}  // namespace wdr_mission
