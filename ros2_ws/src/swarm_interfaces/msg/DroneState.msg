# State of one drone, as seen by the swarm commander.

string id                       # e.g. "cf1"
string status                   # landed | taking_off | flying | landing
geometry_msgs/Point position    # current position in "world" (m)
bool has_target                 # true while a go_to/formation target is active
geometry_msgs/Point target      # last commanded target (valid if has_target)
float32 nearest_neighbour       # distance to the closest other drone (m)
