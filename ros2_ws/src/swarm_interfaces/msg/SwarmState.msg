# Whole swarm in one message. Published on /swarm/state by swarm_telemetry (~10 Hz).

builtin_interfaces/Time stamp
DroneState[] drones                    # one entry per drone, same order every time

# summary for the MR UI (e.g. "4/6 flying · 1 warning · 0 faults")
uint8 num_drones
uint8 num_airborne
uint8 num_warnings                     # drones with warnings != 0
uint8 num_faults                       # drones with faults != 0

float32 min_separation                 # smallest distance between airborne drones (m), 0 if < 2 airborne
bool separation_warning                # min_separation < configured limit
