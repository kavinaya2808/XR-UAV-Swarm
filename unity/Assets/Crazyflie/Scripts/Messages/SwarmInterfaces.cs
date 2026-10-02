// Hand-written ros-sharp bindings for our own swarm_interfaces services (swarm_control / swarm_commander).
//
// The request classes only carry the fields the UI sets. rosbridge fills every missing field with its
// default, so a request here is exactly the same as the terminal calls:
//   ros2 service call /swarm/takeoff   swarm_interfaces/srv/SwarmCommand "{}"
//   ros2 service call /swarm/land      swarm_interfaces/srv/SwarmCommand "{}"
//   ros2 service call /swarm/formation swarm_interfaces/srv/Formation "{shape: circle, center: {x: 1.0, y: 0.5, z: 1.0}}"
// Empty drone_ids = all drones; height / spacing / heading / duration = 0 -> swarm_commander uses its own defaults.
using RosSharp.RosBridgeClient.MessageTypes.BuiltinInterfaces;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.SwarmInterfaces
{
    // srv/SwarmCommand.srv (used by /swarm/takeoff and /swarm/land)
    public class SwarmCommandRequest : Message
    {
        public const string RosMessageName = "swarm_interfaces/srv/SwarmCommand";
    }

    public class SwarmCommandResponse : Message
    {
        public const string RosMessageName = "swarm_interfaces/srv/SwarmCommand";
        public bool success { get; set; }
        public string message { get; set; }
        public SwarmCommandResponse() { success = false; message = ""; }
    }

    // srv/Formation.srv (used by /swarm/formation). shape: line | grid | circle | v
    public class FormationRequest : Message
    {
        public const string RosMessageName = "swarm_interfaces/srv/Formation";
        public string shape { get; set; }
        public Point center { get; set; }

        public FormationRequest() { shape = "line"; center = new Point(); }
        public FormationRequest(string shape, Point center) { this.shape = shape; this.center = center; }
    }

    public class FormationResponse : Message
    {
        public const string RosMessageName = "swarm_interfaces/srv/Formation";
        public bool success { get; set; }
        public string message { get; set; }
        public FormationResponse() { success = false; message = ""; }
    }

    // ---------------------------------------------------------------------------------------------
    // Telemetry (swarm_interfaces 0.2.0), published by swarm_telemetry on /swarm/state at ~10 Hz.
    // Only the fields Unity reads are declared; rosbridge/Newtonsoft ignore the rest.
    // status / flight_mode / nearest_neighbour are kept loose on purpose: Newtonsoft turns a number
    // into a string, and `object` accepts anything, so these classes survive small .msg changes.
    // Interpret them through DroneTelemetry (DroneTelemetry.cs), not directly.
    // ---------------------------------------------------------------------------------------------

    // msg/DroneState.msg
    public class DroneState : Message
    {
        public const string RosMessageName = "swarm_interfaces/msg/DroneState";
        public Time stamp { get; set; }
        public string id { get; set; }
        public Point position { get; set; }          // ROS world frame (x fwd, y left, z up), m
        public Vector3 velocity { get; set; }        // m/s
        public double speed { get; set; }            // m/s
        public double yaw { get; set; }              // rad
        public double battery_percent { get; set; }  // 0..100
        public double battery_voltage { get; set; }  // V (1S LiPo)
        public string flight_mode { get; set; }      // landed | taking_off | hovering | moving | landing
        public string status { get; set; }           // idle | flying | searching | returning | fault
        public long warnings { get; set; }           // bit field, see DroneWarnings
        public long faults { get; set; }             // bit field, see DroneFaults
        public object alerts { get; set; }           // human-readable text (string or list)
        public double link_quality { get; set; }     // 0..1
        public bool has_target { get; set; }
        public Point target { get; set; }
        public object nearest_neighbour { get; set; }

        public DroneState()
        {
            stamp = new Time(); id = ""; position = new Point(); velocity = new Vector3();
            flight_mode = ""; status = ""; target = new Point();
        }
    }

    // msg/SwarmState.msg
    public class SwarmState : Message
    {
        public const string RosMessageName = "swarm_interfaces/msg/SwarmState";
        public Time stamp { get; set; }
        public DroneState[] drones { get; set; }
        public int num_drones { get; set; }
        public int num_airborne { get; set; }
        public int num_warnings { get; set; }
        public int num_faults { get; set; }
        public double min_separation { get; set; }   // m
        public bool separation_warning { get; set; }

        public SwarmState() { stamp = new Time(); drones = new DroneState[0]; }
    }
}
