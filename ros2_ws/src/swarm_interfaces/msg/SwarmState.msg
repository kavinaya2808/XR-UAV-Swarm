# Published on /swarm/state (10 Hz by default).

builtin_interfaces/Time stamp
DroneState[] drones
float32 min_separation          # smallest distance between any two drones (m)
bool separation_warning         # true if min_separation < configured limit
