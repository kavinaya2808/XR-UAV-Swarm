using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;

// Menu: Crazyflie > 1. Build Drone Prefab   -> Assets/Crazyflie/Prefabs/Crazyflie.prefab
//       Crazyflie > 2. Build Simulation Scene -> ROS / World / cf231 / GoalMarker / Ground in the open scene
// Safe to run again: existing objects are reused, the prefab is rebuilt.
public static class CrazyflieSceneBuilder
{
    private const string ModelPath  = "Assets/Crazyflie/Models/cf2_assembly_with_props.dae";
    private const string PrefabPath = "Assets/Crazyflie/Prefabs/Crazyflie.prefab";
    private const string MatDir     = "Assets/Crazyflie/Materials";
    private const string DroneName  = "cf231";

    [MenuItem("Crazyflie/1. Build Drone Prefab")]
    public static GameObject BuildPrefab()
    {
        var bodyMat  = GetMaterial("CF_Body",  new Color(0.15f, 0.15f, 0.17f));
        var frontMat = GetMaterial("CF_Front", new Color(0.9f, 0.15f, 0.1f));

        var root = new GameObject("Crazyflie");
        root.AddComponent<CrazyflieFollower>();
        root.AddComponent<CrazyflieCommander>();

        GameObject visual;
        var model = AssetDatabase.LoadAssetAtPath<GameObject>(ModelPath);
        if (model != null)
        {
            visual = (GameObject)PrefabUtility.InstantiatePrefab(model);
            PrefabUtility.UnpackPrefabInstance(visual, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        }
        else
        {
            Debug.LogWarning($"[Crazyflie] Model not found at {ModelPath}; using a primitive placeholder.");
            visual = BuildPlaceholder();
        }
        visual.name = "Visual";
        visual.transform.SetParent(root.transform, false);
        StripColliders(visual);
        foreach (var r in visual.GetComponentsInChildren<Renderer>()) r.sharedMaterial = bodyMat;
        FitVisual(root.transform, visual.transform);

        // Small red dot on the nose (+Z in Unity = +X in ROS) to check heading.
        var front = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        front.name = "FrontMarker";
        Object.DestroyImmediate(front.GetComponent<Collider>());
        front.transform.SetParent(root.transform, false);
        front.transform.localPosition = new Vector3(0f, 0.012f, 0.035f);
        front.transform.localScale = Vector3.one * 0.012f;
        front.GetComponent<Renderer>().sharedMaterial = frontMat;

        EnsureFolder("Assets/Crazyflie/Prefabs");
        var prefab = PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        Object.DestroyImmediate(root);
        Debug.Log($"[Crazyflie] Prefab saved: {PrefabPath}");
        return prefab;
    }

    [MenuItem("Crazyflie/2. Build Simulation Scene")]
    public static void BuildScene()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null) prefab = BuildPrefab();

        // ROS connection
        var ros = FindOrCreate("ROS", null);
        var rc = ros.GetComponent<RosConnector>();
        if (rc == null) rc = ros.AddComponent<RosConnector>();
        rc.selectedRosVersion = RosVersion.ROS2;
        rc.protocol = Protocol.WebSocketNET;
        rc.Serializer = RosSocket.SerializerEnum.Newtonsoft_JSON;
        if (string.IsNullOrEmpty(rc.RosBridgeServerUrl)) rc.RosBridgeServerUrl = "ws://localhost:9090";
        if (ros.GetComponent<CrazyflieTfListener>() == null) ros.AddComponent<CrazyflieTfListener>();

        // World root = ROS "world" frame
        var world = FindOrCreate("World", null);
        world.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        world.transform.localScale = Vector3.one;

        var volume = FindOrCreate("FlightVolume", world.transform);
        if (volume.GetComponent<FlightVolume>() == null) volume.AddComponent<FlightVolume>();

        // Drone
        var droneT = world.transform.Find(DroneName);
        GameObject drone = droneT != null ? droneT.gameObject : null;
        if (drone == null)
        {
            drone = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            drone.name = DroneName;
            drone.transform.SetParent(world.transform, false);
            drone.transform.localPosition = Vector3.zero;
        }
        var follower = drone.GetComponent<CrazyflieFollower>();
        var commander = drone.GetComponent<CrazyflieCommander>();
        follower.CfName = DroneName;
        commander.CfName = DroneName;

        // Goal marker for "G" (go to)
        var goalT = world.transform.Find("GoalMarker");
        GameObject goal = goalT != null ? goalT.gameObject : null;
        if (goal == null)
        {
            goal = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            goal.name = "GoalMarker";
            Object.DestroyImmediate(goal.GetComponent<Collider>());
            goal.transform.SetParent(world.transform, false);
            goal.transform.localPosition = new Vector3(1f, 1f, 1f);
            goal.transform.localScale = Vector3.one * 0.08f;
            goal.GetComponent<Renderer>().sharedMaterial = GetMaterial("CF_Goal", new Color(1f, 0.6f, 0.1f));
        }
        commander.GoalMarker = goal.transform;
        PrefabUtility.RecordPrefabInstancePropertyModifications(follower);
        PrefabUtility.RecordPrefabInstancePropertyModifications(commander);

        // Ground
        var ground = GameObject.Find("Ground");
        if (ground == null)
        {
            ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "Ground";
            ground.transform.localScale = new Vector3(3f, 1f, 3f);
            ground.GetComponent<Renderer>().sharedMaterial = GetMaterial("CF_Ground", new Color(0.32f, 0.33f, 0.35f));
        }

        // Camera looking at the flight volume
        var cam = Camera.main;
        if (cam != null)
        {
            cam.transform.position = new Vector3(0f, 1.4f, -3f);
            cam.transform.LookAt(new Vector3(0f, 0.6f, 0f));
            cam.nearClipPlane = 0.01f;
        }

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = ros;
        Debug.Log("[Crazyflie] Scene built. Set ROS > RosConnector > Ros Bridge Server Url to ws://<ubuntu-ip>:9090, save the scene, press Play.");
    }

    // Switch the scene from one hand-placed drone to auto-spawned swarm mode.
    [MenuItem("Crazyflie/3. Switch Scene to Swarm Mode")]
    public static void SwitchToSwarm()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null) prefab = BuildPrefab();
        var world = FindOrCreate("World", null);

        // Remove the single hand-placed drone; the spawner creates drones from /tf.
        var single = world.transform.Find(DroneName);
        if (single != null) Undo.DestroyObjectImmediate(single.gameObject);

        var swarm = FindOrCreate("Swarm", world.transform);
        var spawner = swarm.GetComponent<CrazyflieSwarmSpawner>();
        if (spawner == null) spawner = Undo.AddComponent<CrazyflieSwarmSpawner>(swarm);
        spawner.DronePrefab = prefab;
        spawner.WorldRoot = world.transform;
        if (swarm.GetComponent<CrazyflieSwarmCommander>() == null) Undo.AddComponent<CrazyflieSwarmCommander>(swarm);

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = swarm;
        Debug.Log("[Crazyflie] Swarm mode: drones are spawned from /tf at Play. Keys: T = all take off, L = all land, E = emergency.");
    }

    // Place the drones listed on the spawner (mirror of crazyflies.yaml) in the scene, so they are
    // visible and selectable in Edit mode. At Play they are adopted and follow /tf.
    [MenuItem("Crazyflie/4. Place Swarm Drones in Scene")]
    public static void PlaceSwarmDrones()
    {
        var spawner = Object.FindFirstObjectByType<CrazyflieSwarmSpawner>();
        if (spawner == null) { SwitchToSwarm(); spawner = Object.FindFirstObjectByType<CrazyflieSwarmSpawner>(); }
        var prefab = spawner.DronePrefab != null ? spawner.DronePrefab : AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        var world = spawner.WorldRoot != null ? spawner.WorldRoot : FindOrCreate("World", null).transform;

        int placed = 0;
        foreach (var e in spawner.DroneList)
        {
            if (string.IsNullOrEmpty(e.Name)) continue;
            var existing = world.Find(e.Name);
            GameObject go = existing != null ? existing.gameObject : null;
            if (go == null)
            {
                go = (GameObject)PrefabUtility.InstantiatePrefab(prefab, world);
                go.name = e.Name;
                Undo.RegisterCreatedObjectUndo(go, "Place drone");
                placed++;
            }
            go.transform.localPosition = CrazyflieSwarmSpawner.RosToUnity(e.RosInitialPosition);
            go.transform.localRotation = Quaternion.identity;

            var f = go.GetComponent<CrazyflieFollower>();
            f.CfName = e.Name;
            var c = go.GetComponent<CrazyflieCommander>();
            if (c != null) { c.CfName = e.Name; c.EnableKeyboard = false; }
            PrefabUtility.RecordPrefabInstancePropertyModifications(f);
            if (c != null) PrefabUtility.RecordPrefabInstancePropertyModifications(c);
        }

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log($"[Crazyflie] {placed} drone(s) placed under {world.name}. Save the scene (Cmd+S).");
    }

    // ---------- helpers ----------

    // Rotate so the drone is flat (thinnest along Y), scale if units are off, centre at the root.
    private static void FitVisual(Transform root, Transform visual)
    {
        Quaternion[] candidates = { Quaternion.identity, Quaternion.Euler(-90f, 0f, 0f), Quaternion.Euler(90f, 0f, 0f), Quaternion.Euler(0f, 0f, 90f) };
        Quaternion best = Quaternion.identity;
        float bestRatio = float.PositiveInfinity;
        foreach (var c in candidates)
        {
            visual.localRotation = c;
            var b = WorldBounds(visual.gameObject);
            float ratio = b.size.y / Mathf.Max(1e-6f, Mathf.Max(b.size.x, b.size.z));
            if (ratio < bestRatio - 1e-3f) { bestRatio = ratio; best = c; }
        }
        visual.localRotation = best;

        var bounds = WorldBounds(visual.gameObject);
        float span = Mathf.Max(bounds.size.x, bounds.size.z);
        if (span > 1f || span < 0.01f)
        {
            visual.localScale *= 0.1f / span;   // Crazyflie with props is ~10 cm across
            bounds = WorldBounds(visual.gameObject);
        }
        visual.localPosition -= root.InverseTransformPoint(bounds.center);
    }

    private static Bounds WorldBounds(GameObject go)
    {
        var renderers = go.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0) return new Bounds(go.transform.position, Vector3.zero);
        var b = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++) b.Encapsulate(renderers[i].bounds);
        return b;
    }

    private static GameObject BuildPlaceholder()
    {
        var v = new GameObject("Visual");
        var body = GameObject.CreatePrimitive(PrimitiveType.Cube);
        body.transform.SetParent(v.transform, false);
        body.transform.localScale = new Vector3(0.03f, 0.01f, 0.05f);
        float d = 0.033f;
        foreach (var p in new[] { new Vector3(d, 0.006f, d), new Vector3(-d, 0.006f, d), new Vector3(d, 0.006f, -d), new Vector3(-d, 0.006f, -d) })
        {
            var rotor = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            rotor.transform.SetParent(v.transform, false);
            rotor.transform.localPosition = p;
            rotor.transform.localScale = new Vector3(0.047f, 0.001f, 0.047f);
        }
        return v;
    }

    private static void StripColliders(GameObject go)
    {
        foreach (var c in go.GetComponentsInChildren<Collider>()) Object.DestroyImmediate(c);
    }

    private static GameObject FindOrCreate(string name, Transform parent)
    {
        if (parent != null)
        {
            var t = parent.Find(name);
            if (t != null) return t.gameObject;
        }
        else
        {
            foreach (var r in EditorSceneManager.GetActiveScene().GetRootGameObjects())
                if (r.name == name) return r;
        }
        var go = new GameObject(name);
        if (parent != null) go.transform.SetParent(parent, false);
        return go;
    }

    private static Material GetMaterial(string name, Color color)
    {
        EnsureFolder(MatDir);
        string path = $"{MatDir}/{name}.mat";
        var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (mat != null) return mat;
        var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        mat = new Material(shader) { color = color };
        AssetDatabase.CreateAsset(mat, path);
        return mat;
    }

    private static void EnsureFolder(string path)
    {
        if (AssetDatabase.IsValidFolder(path)) return;
        string parent = System.IO.Path.GetDirectoryName(path).Replace('\\', '/');
        EnsureFolder(parent);
        AssetDatabase.CreateFolder(parent, System.IO.Path.GetFileName(path));
    }
}
