using UnityEngine;
using UnityEngine.InputSystem;
using RosSharp.RosBridgeClient;
using CF = RosSharp.RosBridgeClient.MessageTypes.CrazyflieInterfaces;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;

// Sends high-level commands to one Crazyflie through Crazyswarm2's services:
//   /<cf>/takeoff, /<cf>/land, /<cf>/go_to, /<cf>/emergency
// Keyboard (Play mode): T = take off, L = land, G = go to GoalMarker, E = emergency stop.
// The same methods are available from the component's context menu and from code (VR UI later).
public class CrazyflieCommander : MonoBehaviour
{
    [Tooltip("Name from crazyflies.yaml, e.g. cf231.")]
    public string CfName = "cf231";

    [Tooltip("Found automatically if left empty.")]
    public RosConnector Connector;

    [Header("Take off / land")]
    public float TakeoffHeight = 0.5f;
    public float TakeoffDuration = 2.5f;
    public float LandHeight = 0.04f;
    public float LandDuration = 2.5f;

    [Header("Go to")]
    [Tooltip("Target in the same World root as the drone. Its local position is sent as the goal.")]
    public Transform GoalMarker;
    public float GoToDuration = 3f;

    [Header("Input")]
    public bool EnableKeyboard = true;

    private void Start()
    {
        if (Connector == null) Connector = FindFirstObjectByType<RosConnector>();
    }

    private void Update()
    {
        if (!EnableKeyboard || Keyboard.current == null) return;
        var kb = Keyboard.current;
        if (kb.tKey.wasPressedThisFrame) Takeoff();
        if (kb.lKey.wasPressedThisFrame) Land();
        if (kb.gKey.wasPressedThisFrame) GoToMarker();
        if (kb.eKey.wasPressedThisFrame) Emergency();
    }

    [ContextMenu("Take off")]
    public void Takeoff() =>
        Call<CF.TakeoffRequest, CF.TakeoffResponse>("takeoff",
            new CF.TakeoffRequest(0, TakeoffHeight, CF.DurationUtil.FromSeconds(TakeoffDuration)));

    [ContextMenu("Land")]
    public void Land() =>
        Call<CF.LandRequest, CF.LandResponse>("land",
            new CF.LandRequest(0, LandHeight, CF.DurationUtil.FromSeconds(LandDuration)));

    [ContextMenu("Go to marker")]
    public void GoToMarker()
    {
        if (GoalMarker == null) { Debug.LogWarning("[Crazyflie] No GoalMarker assigned.", this); return; }
        GoTo(GoalMarker.localPosition, GoToDuration);
    }

    /// Absolute go-to. unityLocalPosition is relative to the World root (= ROS world frame).
    /// Yaw is kept at 0 for now.
    public void GoTo(Vector3 unityLocalPosition, float duration)
    {
        // Unity (x right, y up, z fwd) -> ROS (x fwd, y left, z up)
        var goal = new Geometry.Point(unityLocalPosition.z, -unityLocalPosition.x, unityLocalPosition.y);
        Call<CF.GoToRequest, CF.GoToResponse>("go_to",
            new CF.GoToRequest(0, false, goal, 0f, CF.DurationUtil.FromSeconds(duration)));
    }

    [ContextMenu("Emergency stop")]
    public void Emergency() =>
        Call<Std.EmptyRequest, Std.EmptyResponse>("emergency", new Std.EmptyRequest());

    private void Call<TReq, TRes>(string service, TReq request) where TReq : Message where TRes : Message
    {
        var socket = Connector != null ? Connector.RosSocket : null;
        if (socket == null) { Debug.LogWarning("[Crazyflie] Not connected to rosbridge.", this); return; }

        string name = $"/{CfName}/{service}";
        socket.CallService<TReq, TRes>(name, _ => Debug.Log($"[Crazyflie] {name} accepted"), request);
        Debug.Log($"[Crazyflie] -> {name}");
    }
}
