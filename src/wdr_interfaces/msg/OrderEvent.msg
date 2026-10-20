# Published on /wdr/order_events whenever an order changes state. Recorded in the rosbag and
# read back by wdr_tools' metrics report, so every order's life can be reconstructed offline.

string QUEUED=queued
string STARTED=started
string STATE=state        # mission state change, see `state`
string SUCCEEDED=succeeded
string FAILED=failed
string CANCELED=canceled
string REJECTED=rejected

builtin_interfaces/Time stamp
string order_id
string pickup_station
string dropoff_station
string event
# Mission states: CHECKING_BATTERY, GOING_TO_CHARGE, CHARGING, UNDOCKING, TO_PICKUP,
# WAITING_FOR_LOAD, TO_DROPOFF, WAITING_FOR_UNLOAD, NAV_RETRY, TO_EXCEPTION_AREA, DONE
string state
uint16 error_code         # DeliverOrder result code for failed/canceled
string detail
float32 battery_percentage
