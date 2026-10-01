using UnityEngine;
using UnityEngine.InputSystem;
using RosSharp.RosBridgeClient;
using CF = RosSharp.RosBridgeClient.MessageTypes.CrazyflieInterfaces;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;

// Swarm-wide commands through Crazyswarm2's broadcast services:
//   /all/takeoff, /all/land, /all/emergency
// Keyboard (Play mode): T = all take off, L = all land, E = emergency stop (all).
public class CrazyflieSwarmCommander : MonoBehaviour
{
    [Tooltip("Found automatically if left empty.")]
    public RosConnector Connector;

    public float TakeoffHeight = 0.5f;
    public float TakeoffDuration = 2.5f;
    public float LandHeight = 0.04f;
    public float LandDuration = 2.5f;
    public bool EnableKeyboard = true;

    private void Start()
    {
        if (Connector == null) Connector = FindFirstObjectByType<RosConnector>();
    }

    private void Update()
    {
        if (!EnableKeyboard || Keyboard.current == null) return;
        var kb = Keyboard.current;
        if (kb.tKey.wasPressedThisFrame) TakeoffAll();
        if (kb.lKey.wasPressedThisFrame) LandAll();
        if (kb.eKey.wasPressedThisFrame) EmergencyAll();
    }

    [ContextMenu("Take off (all)")]
    public void TakeoffAll() =>
        Call<CF.TakeoffRequest, CF.TakeoffResponse>("/all/takeoff",
            new CF.TakeoffRequest(0, TakeoffHeight, CF.DurationUtil.FromSeconds(TakeoffDuration)));

    [ContextMenu("Land (all)")]
    public void LandAll() =>
        Call<CF.LandRequest, CF.LandResponse>("/all/land",
            new CF.LandRequest(0, LandHeight, CF.DurationUtil.FromSeconds(LandDuration)));

    [ContextMenu("Emergency stop (all)")]
    public void EmergencyAll() =>
        Call<Std.EmptyRequest, Std.EmptyResponse>("/all/emergency", new Std.EmptyRequest());

    private void Call<TReq, TRes>(string service, TReq request) where TReq : Message where TRes : Message
    {
        var socket = Connector != null ? Connector.RosSocket : null;
        if (socket == null) { Debug.LogWarning("[Crazyflie] Not connected to rosbridge.", this); return; }
        socket.CallService<TReq, TRes>(service, _ => Debug.Log($"[Crazyflie] {service} accepted"), request);
        Debug.Log($"[Crazyflie] -> {service}");
    }
}
