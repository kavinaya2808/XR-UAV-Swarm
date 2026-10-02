using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SI = RosSharp.RosBridgeClient.MessageTypes.SwarmInterfaces;

// Mirrors the constants in swarm_interfaces/msg/DroneState.msg so the UI never compares raw strings or bits.

public enum DroneStatus { Unknown = -1, Idle = 0, Flying = 1, Searching = 2, Returning = 3, Fault = 4 }

public enum FlightMode { Unknown = -1, Landed = 0, TakingOff = 1, Hovering = 2, Moving = 3, Landing = 4 }

[Flags]
public enum DroneWarnings : long
{
    None = 0, BatteryLow = 1, BatteryCritical = 2, LinkWeak = 4, Separation = 8, Geofence = 16, PositionStale = 32
}

[Flags]
public enum DroneFaults : long
{
    None = 0, BatteryDepleted = 1, LinkLost = 2, Motor = 4, Sensor = 8, PositionLost = 16
}

public static class DroneTelemetry
{
    // ---------- parsing (accepts "searching", "STATUS_SEARCHING" or "2") ----------

    public static DroneStatus Status(SI.DroneState d)
    {
        if (d == null) return DroneStatus.Unknown;
        switch (Norm(d.status, "status_"))
        {
            case "idle": return DroneStatus.Idle;
            case "flying": return DroneStatus.Flying;
            case "searching": return DroneStatus.Searching;
            case "returning": return DroneStatus.Returning;
            case "fault": return DroneStatus.Fault;
        }
        return FromIndex(d.status, DroneStatus.Unknown);
    }

    public static FlightMode Mode(SI.DroneState d)
    {
        if (d == null) return FlightMode.Unknown;
        switch (Norm(d.flight_mode, "mode_"))
        {
            case "landed": return FlightMode.Landed;
            case "taking_off": case "takingoff": return FlightMode.TakingOff;
            case "hovering": return FlightMode.Hovering;
            case "moving": return FlightMode.Moving;
            case "landing": return FlightMode.Landing;
        }
        return FromIndex(d.flight_mode, FlightMode.Unknown);
    }

    public static DroneWarnings Warnings(SI.DroneState d) => d == null ? DroneWarnings.None : (DroneWarnings)d.warnings;
    public static DroneFaults Faults(SI.DroneState d) => d == null ? DroneFaults.None : (DroneFaults)d.faults;

    public static bool HasWarning(SI.DroneState d) => d != null && d.warnings != 0;
    public static bool HasFault(SI.DroneState d) => d != null && (d.faults != 0 || Status(d) == DroneStatus.Fault);

    /// Alerts as a list of lines, whether ROS sent one string or a list.
    public static List<string> Alerts(SI.DroneState d)
    {
        var lines = new List<string>();
        if (d == null || d.alerts == null) return lines;
        if (d.alerts is string s) { if (s.Length > 0) lines.AddRange(s.Split('\n')); }
        else if (d.alerts is IEnumerable e) foreach (var a in e) { var t = a?.ToString(); if (!string.IsNullOrEmpty(t)) lines.Add(t); }
        return lines;
    }

    /// ROS (x fwd, y left, z up) -> Unity (x right, y up, z fwd), in the World root's local space.
    public static Vector3 UnityPosition(SI.DroneState d) =>
        d == null ? Vector3.zero : new Vector3(-(float)d.position.y, (float)d.position.z, (float)d.position.x);

    // ---------- presentation ----------

    public static readonly Color NoData    = new Color(0.30f, 0.30f, 0.32f);
    public static readonly Color IdleCol   = new Color(0.62f, 0.64f, 0.68f);
    public static readonly Color FlyingCol = new Color(0.30f, 0.85f, 0.45f);
    public static readonly Color SearchCol = new Color(0.20f, 0.80f, 1.00f);
    public static readonly Color ReturnCol = new Color(0.70f, 0.52f, 1.00f);
    public static readonly Color FaultCol  = new Color(1.00f, 0.25f, 0.20f);
    public static readonly Color WarnCol   = new Color(1.00f, 0.72f, 0.15f);

    public static Color StatusColor(DroneStatus s)
    {
        switch (s)
        {
            case DroneStatus.Idle: return IdleCol;
            case DroneStatus.Flying: return FlyingCol;
            case DroneStatus.Searching: return SearchCol;
            case DroneStatus.Returning: return ReturnCol;
            case DroneStatus.Fault: return FaultCol;
            default: return NoData;
        }
    }

    public static string Label(DroneStatus s) => s == DroneStatus.Unknown ? "?" : s.ToString().ToLowerInvariant();

    public static string Label(FlightMode m)
    {
        switch (m)
        {
            case FlightMode.TakingOff: return "taking off";
            case FlightMode.Unknown: return "?";
            default: return m.ToString().ToLowerInvariant();
        }
    }

    /// Short names of the active flags, most severe first, e.g. "MOTOR, LINK LOST".
    public static string FaultText(DroneFaults f) => Flags(f, new (long, string)[] {
        ((long)DroneFaults.Motor, "MOTOR"), ((long)DroneFaults.BatteryDepleted, "BATTERY EMPTY"),
        ((long)DroneFaults.LinkLost, "LINK LOST"), ((long)DroneFaults.PositionLost, "POSITION LOST"),
        ((long)DroneFaults.Sensor, "SENSOR") });

    public static string WarningText(DroneWarnings w) => Flags(w, new (long, string)[] {
        ((long)DroneWarnings.BatteryCritical, "battery critical"),
        ((long)(DroneWarnings.BatteryLow), "battery low"),
        ((long)DroneWarnings.Separation, "too close"), ((long)DroneWarnings.Geofence, "geofence"),
        ((long)DroneWarnings.LinkWeak, "weak link"), ((long)DroneWarnings.PositionStale, "position stale") },
        skipLowBatteryIfCritical: true);

    // ---------- helpers ----------

    private static string Flags<T>(T value, (long bit, string name)[] table, bool skipLowBatteryIfCritical = false) where T : Enum
    {
        long v = Convert.ToInt64(value);
        var parts = new List<string>();
        foreach (var (bit, name) in table)
        {
            if ((v & bit) == 0) continue;
            if (skipLowBatteryIfCritical && bit == (long)DroneWarnings.BatteryLow && (v & (long)DroneWarnings.BatteryCritical) != 0) continue;
            parts.Add(name);
        }
        return string.Join(", ", parts);
    }

    private static string Norm(string s, string prefix)
    {
        if (string.IsNullOrEmpty(s)) return "";
        s = s.Trim().ToLowerInvariant();
        return s.StartsWith(prefix) ? s.Substring(prefix.Length) : s;
    }

    private static T FromIndex<T>(string s, T fallback) where T : struct, Enum
    {
        if (int.TryParse(s, out int i) && Enum.IsDefined(typeof(T), i)) return (T)Enum.ToObject(typeof(T), i);
        return fallback;
    }
}
