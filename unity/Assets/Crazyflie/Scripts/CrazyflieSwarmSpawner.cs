using System.Collections.Generic;
using UnityEngine;

// Creates one drone prefab per Crazyflie that appears on /tf (world -> cfX), so the Unity scene
// always matches crazyflies.yaml without listing the drones twice. Existing drones under the
// World root (matched by CfName) are adopted instead of duplicated.
public class CrazyflieSwarmSpawner : MonoBehaviour
{
    [System.Serializable]
    public struct DroneEntry
    {
        public string Name;
        [Tooltip("initial_position from crazyflies.yaml, in ROS coordinates (x fwd, y left, z up).")]
        public Vector3 RosInitialPosition;
    }

    [Tooltip("Mirror of crazyflies.yaml. Used by 'Crazyflie > 4. Place Swarm Drones in Scene' to show the drones in Edit mode. At runtime any extra drone on /tf is still spawned automatically.")]
    public DroneEntry[] DroneList =
    {
        new DroneEntry { Name = "cf1", RosInitialPosition = new Vector3(-1f, -0.5f, 0f) },
        new DroneEntry { Name = "cf2", RosInitialPosition = new Vector3( 0f, -0.5f, 0f) },
        new DroneEntry { Name = "cf3", RosInitialPosition = new Vector3( 1f, -0.5f, 0f) },
        new DroneEntry { Name = "cf4", RosInitialPosition = new Vector3(-1f,  0.5f, 0f) },
        new DroneEntry { Name = "cf5", RosInitialPosition = new Vector3( 0f,  0.5f, 0f) },
        new DroneEntry { Name = "cf6", RosInitialPosition = new Vector3( 1f,  0.5f, 0f) },
    };

    public static Vector3 RosToUnity(Vector3 ros) => new Vector3(-ros.y, ros.z, ros.x);

    [Tooltip("Assets/Crazyflie/Prefabs/Crazyflie.prefab")]
    public GameObject DronePrefab;

    [Tooltip("Parent for spawned drones (= ROS world frame). Defaults to this object's parent.")]
    public Transform WorldRoot;

    [Tooltip("Found automatically if left empty.")]
    public CrazyflieTfListener Listener;

    [Tooltip("How often to look for new drone names (s).")]
    public float ScanInterval = 0.5f;

    private readonly Dictionary<string, CrazyflieFollower> drones = new Dictionary<string, CrazyflieFollower>();
    private float nextScan;

    public IReadOnlyDictionary<string, CrazyflieFollower> Drones => drones;

    private void Start()
    {
        if (Listener == null) Listener = FindFirstObjectByType<CrazyflieTfListener>();
        if (WorldRoot == null) WorldRoot = transform.parent != null ? transform.parent : transform;
        foreach (var f in WorldRoot.GetComponentsInChildren<CrazyflieFollower>(true))
        {
            drones[f.CfName] = f;
            if (f.Listener == null) f.Listener = Listener;
            var c = f.GetComponent<CrazyflieCommander>();
            if (c != null) c.EnableKeyboard = false;   // keys go to the whole swarm
        }
    }

    private void Update()
    {
        if (Listener == null || DronePrefab == null || Time.time < nextScan) return;
        nextScan = Time.time + ScanInterval;

        foreach (var name in Listener.KnownDrones)
            if (!drones.ContainsKey(name)) Spawn(name);
    }

    private void Spawn(string cfName)
    {
        var go = Instantiate(DronePrefab, WorldRoot);
        go.name = cfName;

        var follower = go.GetComponent<CrazyflieFollower>();
        follower.CfName = cfName;
        follower.Listener = Listener;

        var commander = go.GetComponent<CrazyflieCommander>();
        if (commander != null)
        {
            commander.CfName = cfName;
            commander.EnableKeyboard = false;   // swarm-wide keys live on CrazyflieSwarmCommander
        }

        drones[cfName] = follower;
        Debug.Log($"[Crazyflie] Spawned {cfName} ({drones.Count} drones)");
    }
}
