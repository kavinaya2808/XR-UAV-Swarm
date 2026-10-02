using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using RosSharp.RosBridgeClient;
using SI = RosSharp.RosBridgeClient.MessageTypes.SwarmInterfaces;

// One /swarm/state subscription for the whole scene (published by swarm_telemetry at ~10 Hz).
// Keeps the latest SwarmState and a per-drone lookup for the panel, the drone indicators and,
// later, the MR detail views. Put it next to the RosConnector (SwarmControlPanel adds it if missing).
public class SwarmTelemetryListener : MonoBehaviour
{
    [Tooltip("Found automatically if left empty.")]
    public RosConnector Connector;

    public string StateTopic = "/swarm/state";

    [Tooltip("No message for this long -> telemetry counts as lost (node stopped / rosbridge down).")]
    public float StaleAfterSeconds = 1.5f;

    private readonly object gate = new object();
    private readonly Stopwatch clock = Stopwatch.StartNew();
    private readonly Dictionary<string, SI.DroneState> drones = new Dictionary<string, SI.DroneState>();
    private SI.SwarmState latest;
    private double receivedAt = double.NegativeInfinity;
    private bool subscribed;

    /// Seconds since the last /swarm/state (infinity before the first one).
    public double Age { get { lock (gate) return clock.Elapsed.TotalSeconds - receivedAt; } }
    public bool HasData => Age <= StaleAfterSeconds;

    /// Latest swarm summary (null before the first message).
    public SI.SwarmState Latest { get { lock (gate) return latest; } }

    private void Start()
    {
        if (Connector == null) Connector = GetComponent<RosConnector>();
        if (Connector == null) Connector = FindFirstObjectByType<RosConnector>();
    }

    private void Update()
    {
        if (subscribed || Connector == null || Connector.RosSocket == null) return;
        Connector.RosSocket.Subscribe<SI.SwarmState>(StateTopic, OnState);
        subscribed = true;
        UnityEngine.Debug.Log($"[Telemetry] Subscribed to {StateTopic}");
    }

    // WebSocket thread: no Unity API here. Messages are replaced, never edited, so readers can keep a reference.
    private void OnState(SI.SwarmState msg)
    {
        lock (gate)
        {
            latest = msg;
            receivedAt = clock.Elapsed.TotalSeconds;
            drones.Clear();
            if (msg.drones != null)
                foreach (var d in msg.drones)
                    if (d != null && !string.IsNullOrEmpty(d.id)) drones[d.id.TrimStart('/')] = d;
        }
    }

    /// Latest state of one drone. False if unknown or telemetry is stale.
    public bool TryGetDrone(string cfName, out SI.DroneState state)
    {
        lock (gate) drones.TryGetValue(cfName, out state);
        if (state == null || !HasData) { state = null; return false; }
        return true;
    }

    public IReadOnlyCollection<string> KnownDrones { get { lock (gate) return new List<string>(drones.Keys); } }
}
