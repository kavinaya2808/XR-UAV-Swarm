using System.Collections.Concurrent;
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using SI = RosSharp.RosBridgeClient.MessageTypes.SwarmInterfaces;

// Simple swarm menu: Take Off, Land and the 4 formations.
// Every button calls the same swarm_commander services as the terminal (/swarm/takeoff, /swarm/land,
// /swarm/formation), so swarm_control needs no new logic. The status line shows what ROS answered:
//   "Formation: Line …"  sent, waiting       "Formation: Line ✓"  swarm_commander accepted it
//   "Formation: Line ✗ <reason>"  refused (e.g. separation / not flying)    "… no reply"  timeout
// Works with the mouse in the Editor and with XR ray interactors on a world-space canvas (see the builder).
// Second line (TelemetryText) = live swarm summary from /swarm/state, e.g. "4/6 airborne · 1 warning · min 0.82 m".
public class SwarmControlPanel : MonoBehaviour
{
    [Tooltip("Found automatically if left empty.")]
    public RosConnector Connector;

    [Header("Services (swarm_commander)")]
    public string TakeoffService = "/swarm/takeoff";
    public string LandService = "/swarm/land";
    public string FormationService = "/swarm/formation";

    [Header("Formation")]
    [Tooltip("Formation centre in the ROS world frame (x fwd, y left, z up), metres.")]
    public Vector3 FormationCenterRos = new Vector3(1.0f, 0.5f, 1.0f);

    [Header("UI")]
    public Text StatusText;
    [Tooltip("Seconds without an answer before the status shows 'no reply'.")]
    public float ReplyTimeout = 5f;

    [Header("Telemetry (/swarm/state)")]
    [Tooltip("Found automatically (or added next to the RosConnector) if left empty.")]
    public SwarmTelemetryListener Telemetry;
    public Text TelemetryText;
    [Tooltip("Refresh interval of the summary line (s).")]
    public float TelemetryRefresh = 0.2f;
    private float nextTelemetry;

    private static readonly Color Pending = new Color(1f, 0.78f, 0.3f);
    private static readonly Color Ok = new Color(0.45f, 0.9f, 0.5f);
    private static readonly Color Fail = new Color(1f, 0.45f, 0.4f);

    // rosbridge answers on a background thread -> hand results to the main thread.
    private readonly ConcurrentQueue<(int id, bool ok, string msg)> replies = new ConcurrentQueue<(int, bool, string)>();
    private int commandId;
    private string currentLabel;
    private float sentAt = -1f;

    private void Start()
    {
        if (Connector == null) Connector = FindFirstObjectByType<RosConnector>();
        if (Telemetry == null) Telemetry = FindFirstObjectByType<SwarmTelemetryListener>();
        if (Telemetry == null && Connector != null) Telemetry = Connector.gameObject.AddComponent<SwarmTelemetryListener>();
        SetStatus("Ready", Color.white);
    }

    // ---------- button handlers ----------

    public void TakeOff() =>
        Send<SI.SwarmCommandRequest, SI.SwarmCommandResponse>("Take off", TakeoffService,
            new SI.SwarmCommandRequest(), r => (r.success, r.message));

    public void Land() =>
        Send<SI.SwarmCommandRequest, SI.SwarmCommandResponse>("Land", LandService,
            new SI.SwarmCommandRequest(), r => (r.success, r.message));

    /// shape: line | grid | circle | v
    public void Formation(string shape)
    {
        var c = FormationCenterRos;
        Send<SI.FormationRequest, SI.FormationResponse>($"Formation: {Pretty(shape)}", FormationService,
            new SI.FormationRequest(shape, new Geometry.Point(c.x, c.y, c.z)), r => (r.success, r.message));
    }

    public void FormationLine() => Formation("line");
    public void FormationGrid() => Formation("grid");
    public void FormationCircle() => Formation("circle");
    public void FormationV() => Formation("v");

    // ---------- plumbing ----------

    private void Send<TReq, TRes>(string label, string service, TReq request, System.Func<TRes, (bool, string)> read)
        where TReq : Message where TRes : Message
    {
        var socket = Connector != null ? Connector.RosSocket : null;
        if (socket == null) { SetStatus($"{label} ✗ not connected to rosbridge", Fail); return; }

        int id = ++commandId;              // only the latest command updates the status line
        currentLabel = label;
        sentAt = Time.time;
        SetStatus($"{label} …", Pending);

        socket.CallService<TReq, TRes>(service, res =>
        {
            var (ok, msg) = read(res);
            replies.Enqueue((id, ok, msg));
        }, request);
        Debug.Log($"[Swarm UI] -> {service} ({label})");
    }

    private void Update()
    {
        while (replies.TryDequeue(out var r))
        {
            if (r.id != commandId) continue;
            sentAt = -1f;
            if (r.ok) SetStatus($"{currentLabel} ✓", Ok);
            else SetStatus($"{currentLabel} ✗ {r.msg}", Fail);
            Debug.Log($"[Swarm UI] <- {currentLabel}: {(r.ok ? "ok" : "refused")} {r.msg}");
        }

        if (sentAt >= 0f && Time.time - sentAt > ReplyTimeout)
        {
            sentAt = -1f;
            SetStatus($"{currentLabel} ✗ no reply (is swarm_commander running?)", Fail);
        }

        if (TelemetryText != null && Time.time >= nextTelemetry)
        {
            nextTelemetry = Time.time + TelemetryRefresh;
            UpdateTelemetryLine();
        }
    }

    // "4/6 airborne · 1 warning · 1 fault · min 0.82 m"   red if any fault, amber if any warning.
    private void UpdateTelemetryLine()
    {
        var s = Telemetry != null ? Telemetry.Latest : null;
        if (s == null || !Telemetry.HasData)
        {
            TelemetryText.text = s == null ? "Telemetry: waiting for /swarm/state …" : "Telemetry: lost (is swarm_telemetry running?)";
            TelemetryText.color = new Color(1f, 1f, 1f, 0.5f);
            return;
        }

        int total = s.num_drones > 0 ? s.num_drones : (s.drones != null ? s.drones.Length : 0);
        string text = $"{s.num_airborne}/{total} airborne";
        if (s.num_warnings > 0) text += $" · {s.num_warnings} warning{(s.num_warnings == 1 ? "" : "s")}";
        if (s.num_faults > 0) text += $" · {s.num_faults} fault{(s.num_faults == 1 ? "" : "s")}";
        if (s.num_airborne >= 2 && s.min_separation > 0 && s.min_separation < 100)
            text += $" · min {s.min_separation:0.00} m";
        if (s.separation_warning) text += " · TOO CLOSE";

        TelemetryText.text = text;
        TelemetryText.color = s.num_faults > 0 ? Fail : (s.num_warnings > 0 || s.separation_warning) ? Pending : Ok;
    }

    private void SetStatus(string text, Color color)
    {
        if (StatusText == null) return;
        StatusText.text = text;
        StatusText.color = color;
    }

    private static string Pretty(string shape) =>
        shape == "v" ? "V" : char.ToUpper(shape[0]) + shape.Substring(1);
}
