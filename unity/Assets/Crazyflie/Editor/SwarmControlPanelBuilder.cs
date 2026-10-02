using System;
using UnityEditor;
using UnityEditor.Events;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Events;
using UnityEngine.UI;

// Menu: Crazyflie > 5. Build Swarm Control Panel
// Builds a world-space panel (Take Off, Land, Line, Grid, Circle, V + status line + telemetry line) wired to SwarmControlPanel.
// World-space so it works on a VR headset as well as with the mouse in the Game view.
// If the XR Interaction Toolkit is installed, the XR raycaster / input module are added automatically,
// so controller and hand rays can press the buttons. Safe to run again: the old panel is replaced.
public static class SwarmControlPanelBuilder
{
    private const string PanelName = "SwarmControlPanel";
    private const float PixelsPerMetre = 1000f;       // 1 px = 1 mm -> panel is 42 x 40 cm

    private static readonly Color PanelColor  = new Color(0.08f, 0.09f, 0.11f, 0.88f);
    private static readonly Color ActionColor = new Color(0.20f, 0.42f, 0.78f);
    private static readonly Color LandColor   = new Color(0.62f, 0.30f, 0.22f);
    private static readonly Color ShapeColor  = new Color(0.22f, 0.24f, 0.28f);

    private const string XrRaycaster   = "UnityEngine.XR.Interaction.Toolkit.UI.TrackedDeviceGraphicRaycaster, Unity.XR.Interaction.Toolkit";
    private const string XrInputModule = "UnityEngine.XR.Interaction.Toolkit.UI.XRUIInputModule, Unity.XR.Interaction.Toolkit";

    [MenuItem("Crazyflie/5. Build Swarm Control Panel")]
    public static void Build()
    {
        var old = GameObject.Find(PanelName);
        if (old != null) Undo.DestroyObjectImmediate(old);

        EnsureEventSystem();
        var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");

        // ---- canvas (world space) ----
        var go = new GameObject(PanelName, typeof(RectTransform));
        Undo.RegisterCreatedObjectUndo(go, "Build Swarm Control Panel");
        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.worldCamera = Camera.main;                  // needed for mouse clicks on a world-space canvas
        go.AddComponent<CanvasScaler>().dynamicPixelsPerUnit = 3f;   // sharper text in VR
        go.AddComponent<GraphicRaycaster>();
        AddIfExists(go, XrRaycaster);

        var rt = (RectTransform)go.transform;
        rt.sizeDelta = new Vector2(420, 400);
        rt.localScale = Vector3.one / PixelsPerMetre;
        PlaceInFrontOfCamera(rt);

        var bg = go.AddComponent<Image>();
        bg.color = PanelColor;

        var vlg = go.AddComponent<VerticalLayoutGroup>();
        vlg.padding = new RectOffset(20, 20, 16, 16);
        vlg.spacing = 12;
        vlg.childControlWidth = vlg.childControlHeight = true;
        vlg.childForceExpandWidth = true;
        vlg.childForceExpandHeight = false;

        var panel = go.AddComponent<SwarmControlPanel>();

        // ---- title ----
        var title = MakeText("Title", go.transform, font, "Swarm Control", 30, FontStyle.Bold, TextAnchor.MiddleLeft);
        Fixed(title.gameObject, 38);

        // ---- Take Off / Land ----
        var row = Row("FlightRow", go.transform, 64);
        MakeButton(row, font, "Take Off", ActionColor, panel.TakeOff);
        MakeButton(row, font, "Land", LandColor, panel.Land);

        // ---- formations ----
        var label = MakeText("FormationLabel", go.transform, font, "Formation", 20, FontStyle.Normal, TextAnchor.LowerLeft);
        label.color = new Color(1f, 1f, 1f, 0.6f);
        Fixed(label.gameObject, 24);

        var grid = new GameObject("Formations", typeof(RectTransform));
        grid.transform.SetParent(go.transform, false);
        var glg = grid.AddComponent<GridLayoutGroup>();
        glg.cellSize = new Vector2(184, 52);
        glg.spacing = new Vector2(12, 10);
        glg.constraint = GridLayoutGroup.Constraint.FixedColumnCount;
        glg.constraintCount = 2;
        Fixed(grid, 114);
        MakeButton(grid.transform, font, "Line", ShapeColor, panel.FormationLine);
        MakeButton(grid.transform, font, "Grid", ShapeColor, panel.FormationGrid);
        MakeButton(grid.transform, font, "Circle", ShapeColor, panel.FormationCircle);
        MakeButton(grid.transform, font, "V", ShapeColor, panel.FormationV);

        // ---- status line ----
        var status = MakeText("Status", go.transform, font, "Ready", 20, FontStyle.Normal, TextAnchor.MiddleLeft);
        status.horizontalOverflow = HorizontalWrapMode.Wrap;
        status.verticalOverflow = VerticalWrapMode.Truncate;
        Fixed(status.gameObject, 28);
        panel.StatusText = status;

        // ---- telemetry summary (/swarm/state) ----
        var telemetry = MakeText("Telemetry", go.transform, font, "Telemetry: waiting for /swarm/state …", 20, FontStyle.Normal, TextAnchor.MiddleLeft);
        telemetry.color = new Color(1f, 1f, 1f, 0.5f);
        telemetry.horizontalOverflow = HorizontalWrapMode.Wrap;
        telemetry.verticalOverflow = VerticalWrapMode.Truncate;
        Fixed(telemetry.gameObject, 28);
        panel.TelemetryText = telemetry;
        var ros = GameObject.Find("ROS");
        if (ros != null && ros.GetComponent<SwarmTelemetryListener>() == null) Undo.AddComponent<SwarmTelemetryListener>(ros);

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = go;
        Debug.Log("[Swarm UI] Panel built. Press Play and click the buttons (mouse in the Game view, or XR rays on a headset). Save the scene (Cmd+S).");
    }

    // ---------- helpers ----------

    private static void EnsureEventSystem()
    {
        var es = UnityEngine.Object.FindFirstObjectByType<EventSystem>();
        if (es == null)
        {
            var esGo = new GameObject("EventSystem", typeof(EventSystem));
            Undo.RegisterCreatedObjectUndo(esGo, "Create EventSystem");
            es = esGo.GetComponent<EventSystem>();
        }
        // XR UI module handles mouse + XR rays; otherwise the Input System module (project uses the new Input System only).
        bool hasXr = Type.GetType(XrInputModule) != null;
        if (hasXr) { if (es.GetComponent(Type.GetType(XrInputModule)) == null) AddIfExists(es.gameObject, XrInputModule); }
        else if (es.GetComponent<UnityEngine.InputSystem.UI.InputSystemUIInputModule>() == null)
            Undo.AddComponent<UnityEngine.InputSystem.UI.InputSystemUIInputModule>(es.gameObject);
    }

    private static void AddIfExists(GameObject go, string assemblyQualifiedName)
    {
        var t = Type.GetType(assemblyQualifiedName);
        if (t != null && go.GetComponent(t) == null) Undo.AddComponent(go, t);
    }

    // ~0.9 m in front of the camera, a bit to the left and lower, turned to face it.
    private static void PlaceInFrontOfCamera(RectTransform rt)
    {
        var cam = Camera.main;
        if (cam == null) { rt.position = new Vector3(-0.5f, 1.2f, -2.2f); return; }
        var fwd = Vector3.ProjectOnPlane(cam.transform.forward, Vector3.up).normalized;
        if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.forward;
        var right = Vector3.Cross(Vector3.up, fwd);
        rt.position = cam.transform.position + fwd * 0.9f - right * 0.45f - Vector3.up * 0.2f;
        rt.rotation = Quaternion.LookRotation(rt.position - cam.transform.position, Vector3.up);
    }

    private static Transform Row(string name, Transform parent, float height)
    {
        var row = new GameObject(name, typeof(RectTransform));
        row.transform.SetParent(parent, false);
        var h = row.AddComponent<HorizontalLayoutGroup>();
        h.spacing = 12;
        h.childControlWidth = h.childControlHeight = true;
        h.childForceExpandWidth = h.childForceExpandHeight = true;
        Fixed(row, height);
        return row.transform;
    }

    private static void Fixed(GameObject go, float height)
    {
        var le = go.GetComponent<LayoutElement>();
        if (le == null) le = go.AddComponent<LayoutElement>();
        le.minHeight = le.preferredHeight = height;
    }

    private static Text MakeText(string name, Transform parent, Font font, string text, int size, FontStyle style, TextAnchor anchor)
    {
        var go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var t = go.AddComponent<Text>();
        t.font = font;
        t.text = text;
        t.fontSize = size;
        t.fontStyle = style;
        t.alignment = anchor;
        t.color = Color.white;
        t.raycastTarget = false;
        return t;
    }

    private static void MakeButton(Transform parent, Font font, string label, Color color, UnityAction onClick)
    {
        var go = new GameObject(label, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var img = go.AddComponent<Image>();
        img.color = Color.white;
        var btn = go.AddComponent<Button>();
        var cb = btn.colors;
        cb.normalColor = color;
        cb.highlightedColor = Color.Lerp(color, Color.white, 0.25f);
        cb.pressedColor = Color.Lerp(color, Color.black, 0.25f);
        cb.selectedColor = color;
        btn.colors = cb;
        UnityEventTools.AddPersistentListener(btn.onClick, onClick);

        var txt = MakeText("Label", go.transform, font, label, 24, FontStyle.Bold, TextAnchor.MiddleCenter);
        var trt = (RectTransform)txt.transform;
        trt.anchorMin = Vector2.zero;
        trt.anchorMax = Vector2.one;
        trt.offsetMin = trt.offsetMax = Vector2.zero;
    }
}
