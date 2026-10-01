using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Tf2 = RosSharp.RosBridgeClient.MessageTypes.Tf2;

// One /tf subscription for the whole scene. Crazyswarm2's crazyflie_sim publishes
// world -> <cf name> for every drone (rviz visualization plugin, enabled by default).
// Stores the latest pose per drone (in ROS coordinates) for CrazyflieFollower to read.
// Put this on the same GameObject as the RosConnector.
[RequireComponent(typeof(RosConnector))]
public class CrazyflieTfListener : MonoBehaviour
{
    [Tooltip("TF topic published by crazyflie_sim.")]
    public string TfTopic = "/tf";

    [Tooltip("Parent frame of the drone transforms (crazyflies.yaml: reference_frame).")]
    public string ReferenceFrame = "world";

    [Tooltip("rosbridge throttle in ms. The sim can publish /tf much faster than Unity renders; 10 ms = max 100 Hz.")]
    public int ThrottleMs = 10;

    public struct RosPose
    {
        public Vector3 Position;      // ROS frame: x forward, y left, z up (metres)
        public Quaternion Rotation;   // ROS frame quaternion
        public double ReceivedAt;     // seconds on the listener's stopwatch
    }

    private readonly Dictionary<string, RosPose> latest = new Dictionary<string, RosPose>();
    private readonly object gate = new object();
    private readonly Stopwatch clock = Stopwatch.StartNew();
    private RosConnector connector;
    private bool subscribed;

    public double Now => clock.Elapsed.TotalSeconds;

    private void Awake()
    {
        connector = GetComponent<RosConnector>();
    }

    private void Update()
    {
        if (subscribed || connector == null || connector.RosSocket == null) return;
        connector.RosSocket.Subscribe<Tf2.TFMessage>(TfTopic, OnTf, ThrottleMs);
        subscribed = true;
    }

    // Called on the WebSocket thread: no Unity API here.
    private void OnTf(Tf2.TFMessage msg)
    {
        double now = Now;
        lock (gate)
        {
            foreach (var t in msg.transforms)
            {
                if (Strip(t.header.frame_id) != ReferenceFrame) continue;
                var p = t.transform.translation;
                var q = t.transform.rotation;
                latest[Strip(t.child_frame_id)] = new RosPose
                {
                    Position = new Vector3((float)p.x, (float)p.y, (float)p.z),
                    Rotation = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w),
                    ReceivedAt = now
                };
            }
        }
    }

    /// Latest pose of a drone, already converted to Unity coordinates.
    public bool TryGetPose(string cfName, out Vector3 unityPosition, out Quaternion unityRotation, out double ageSeconds)
    {
        RosPose pose;
        bool found;
        lock (gate) found = latest.TryGetValue(cfName, out pose);

        if (!found)
        {
            unityPosition = Vector3.zero;
            unityRotation = Quaternion.identity;
            ageSeconds = double.PositiveInfinity;
            return false;
        }

        // ROS (x fwd, y left, z up) -> Unity (x right, y up, z fwd); same as ros-sharp's Ros2Unity().
        unityPosition = new Vector3(-pose.Position.y, pose.Position.z, pose.Position.x);
        unityRotation = new Quaternion(pose.Rotation.y, -pose.Rotation.z, -pose.Rotation.x, pose.Rotation.w);
        ageSeconds = Now - pose.ReceivedAt;
        return true;
    }

    public IReadOnlyCollection<string> KnownDrones
    {
        get { lock (gate) return new List<string>(latest.Keys); }
    }

    private static string Strip(string frame) => string.IsNullOrEmpty(frame) ? "" : frame.TrimStart('/');
}
