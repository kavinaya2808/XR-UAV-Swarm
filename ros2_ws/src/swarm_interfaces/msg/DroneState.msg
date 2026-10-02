# Full state of one drone — the single per-drone record that the MR UI,
# search planner and detection pipeline read.
# Published (inside SwarmState) on /swarm/state by swarm_telemetry, ~10 Hz.

# ---------------------------------------------------------------- constants
# status: what the drone is doing at mission level
string STATUS_IDLE="idle"              # on the ground, ready
string STATUS_FLYING="flying"          # airborne, no mission task
string STATUS_SEARCHING="searching"    # airborne, executing a search task
string STATUS_RETURNING="returning"    # airborne, flying back / to land
string STATUS_FAULT="fault"            # at least one fault flag is set

# flight_mode: what the vehicle is physically doing
string MODE_LANDED="landed"
string MODE_TAKING_OFF="taking_off"
string MODE_HOVERING="hovering"
string MODE_MOVING="moving"
string MODE_LANDING="landing"

# warnings (bitmask) — drone still usable, operator should look
uint32 WARN_BATTERY_LOW=1              # battery_percent < battery_low
uint32 WARN_BATTERY_CRITICAL=2         # battery_percent < battery_critical -> land now
uint32 WARN_LINK_WEAK=4                # link_quality < link_weak
uint32 WARN_SEPARATION=8               # nearest airborne neighbour < min_separation
uint32 WARN_GEOFENCE=16                # position outside the flight area
uint32 WARN_POSITION_STALE=32          # no position update for > stale_timeout

# faults (bitmask) — drone must not fly / should be landed
uint32 FAULT_BATTERY_DEPLETED=1        # battery_percent <= battery_depleted
uint32 FAULT_LINK_LOST=2               # no link to the drone
uint32 FAULT_MOTOR=4                   # motor / propulsion failure
uint32 FAULT_SENSOR=8                  # IMU / state-estimator failure
uint32 FAULT_POSITION_LOST=16          # no position for > lost_timeout

# ------------------------------------------------------------------- fields
builtin_interfaces/Time stamp          # time of the data in this record
string id                              # e.g. "cf1"

geometry_msgs/Point position           # world frame (m)
geometry_msgs/Vector3 velocity         # world frame (m/s)
float32 speed                          # |velocity| (m/s)
float32 yaw                            # heading (rad)

float32 battery_percent                # 0..100
float32 battery_voltage                # V (1S LiPo, 3.0..4.2)

string flight_mode                     # MODE_*
string status                          # STATUS_*

uint32 warnings                        # WARN_* bits
uint32 faults                          # FAULT_* bits
string[] alerts                        # human-readable text for every active bit

float32 link_quality                   # 0..1 (1 = perfect)

bool has_target                        # a go_to / formation target is active
geometry_msgs/Point target              # valid if has_target
float32 nearest_neighbour              # distance to the closest airborne other drone (m), 0 = none
