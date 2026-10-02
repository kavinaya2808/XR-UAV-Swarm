# What swarm_commander knows about one drone (its command view).
# Merged into DroneState by swarm_telemetry; the UI should read /swarm/state instead.

string id
string phase                           # landed | taking_off | flying | landing
bool has_target
geometry_msgs/Point target
