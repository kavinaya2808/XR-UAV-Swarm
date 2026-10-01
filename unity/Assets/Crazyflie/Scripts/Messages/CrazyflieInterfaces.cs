// Hand-written ros-sharp bindings for the Crazyswarm2 services used from Unity.
// Source definitions: crazyswarm2/crazyflie_interfaces/srv/{Takeoff,Land,GoTo}.srv
// Request and response share the service type name, as in ros-sharp's generated srv classes.
using RosSharp.RosBridgeClient.MessageTypes.BuiltinInterfaces;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.CrazyflieInterfaces
{
    public static class DurationUtil
    {
        public static Duration FromSeconds(float seconds)
        {
            int sec = (int)System.Math.Floor(seconds);
            uint nanosec = (uint)System.Math.Round((seconds - sec) * 1e9);
            return new Duration(sec, nanosec);
        }
    }

    // uint8 group_mask / float32 height / builtin_interfaces/Duration duration
    public class TakeoffRequest : Message
    {
        public const string RosMessageName = "crazyflie_interfaces/srv/Takeoff";
        public byte group_mask { get; set; }
        public float height { get; set; }
        public Duration duration { get; set; }

        public TakeoffRequest() { group_mask = 0; height = 0f; duration = new Duration(); }
        public TakeoffRequest(byte groupMask, float height, Duration duration)
        { group_mask = groupMask; this.height = height; this.duration = duration; }
    }

    public class TakeoffResponse : Message
    {
        public const string RosMessageName = "crazyflie_interfaces/srv/Takeoff";
    }

    // uint8 group_mask / float32 height / builtin_interfaces/Duration duration
    public class LandRequest : Message
    {
        public const string RosMessageName = "crazyflie_interfaces/srv/Land";
        public byte group_mask { get; set; }
        public float height { get; set; }
        public Duration duration { get; set; }

        public LandRequest() { group_mask = 0; height = 0f; duration = new Duration(); }
        public LandRequest(byte groupMask, float height, Duration duration)
        { group_mask = groupMask; this.height = height; this.duration = duration; }
    }

    public class LandResponse : Message
    {
        public const string RosMessageName = "crazyflie_interfaces/srv/Land";
    }

    // uint8 group_mask / bool relative / geometry_msgs/Point goal / float32 yaw / builtin_interfaces/Duration duration
    public class GoToRequest : Message
    {
        public const string RosMessageName = "crazyflie_interfaces/srv/GoTo";
        public byte group_mask { get; set; }
        public bool relative { get; set; }
        public Point goal { get; set; }
        public float yaw { get; set; }
        public Duration duration { get; set; }

        public GoToRequest() { group_mask = 0; relative = false; goal = new Point(); yaw = 0f; duration = new Duration(); }
        public GoToRequest(byte groupMask, bool relative, Point goal, float yaw, Duration duration)
        { group_mask = groupMask; this.relative = relative; this.goal = goal; this.yaw = yaw; this.duration = duration; }
    }

    public class GoToResponse : Message
    {
        public const string RosMessageName = "crazyflie_interfaces/srv/GoTo";
    }
}
