using UnityEngine;
using SI = RosSharp.RosBridgeClient.MessageTypes.SwarmInterfaces;

// Fixed level of detail for every drone (same for all, adaptive LoD comes later):
//   - a small status light above the drone, coloured by status
//       grey idle · green flying · cyan searching · violet returning · red fault (blinks)
//       a warning makes the light pulse amber on top of its status colour
//   - a label that faces the camera:  "cf3  72%"  +  one line of what matters most
//       fault names > warning names > flight mode
// Added automatically to every drone by CrazyflieSwarmSpawner. Reads SwarmTelemetryListener only.
[RequireComponent(typeof(CrazyflieFollower))]
public class DroneStatusIndicator : MonoBehaviour
{
    [Tooltip("Defaults to CrazyflieFollower.CfName.")]
    public string CfName;

    [Tooltip("Found automatically if left empty.")]
    public SwarmTelemetryListener Telemetry;

    public bool ShowLabel = true;
    [Tooltip("Height of the status light above the drone centre (m).")]
    public float LightHeight = 0.04f;
    public float LightSize = 0.025f;
    public float LabelHeight = 0.075f;
    [Tooltip("Label line height in metres.")]
    public float LabelLineHeight = 0.018f;

    /// Latest telemetry for this drone (null when unknown / stale). For the MR detail panel later.
    public SI.DroneState Current { get; private set; }

    private Renderer lightRenderer;
    private MaterialPropertyBlock block;
    private TextMesh label;
    private Transform labelT;
    private Camera cam;

    private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
    private static readonly int ColorId = Shader.PropertyToID("_Color");
    private static Material sharedLightMaterial;

    private void Start()
    {
        if (string.IsNullOrEmpty(CfName)) CfName = GetComponent<CrazyflieFollower>().CfName;
        if (Telemetry == null) Telemetry = FindFirstObjectByType<SwarmTelemetryListener>();
        BuildVisuals();
    }

    private void BuildVisuals()
    {
        var lightGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        lightGo.name = "StatusLight";
        Destroy(lightGo.GetComponent<Collider>());
        lightGo.transform.SetParent(transform, false);
        lightGo.transform.localPosition = new Vector3(0f, LightHeight, 0f);
        lightGo.transform.localScale = Vector3.one * LightSize;
        lightRenderer = lightGo.GetComponent<Renderer>();
        lightRenderer.sharedMaterial = LightMaterial();
        lightRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        block = new MaterialPropertyBlock();

        if (!ShowLabel) return;
        var labelGo = new GameObject("StatusLabel");
        labelT = labelGo.transform;
        labelT.SetParent(transform, false);
        labelT.localPosition = new Vector3(0f, LabelHeight, 0f);
        label = labelGo.AddComponent<TextMesh>();
        var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        label.font = font;
        labelGo.GetComponent<MeshRenderer>().sharedMaterial = font.material;
        label.fontSize = 64;
        label.characterSize = LabelLineHeight * 10f / 64f;
        label.anchor = TextAnchor.LowerCenter;
        label.alignment = TextAlignment.Center;
        label.text = CfName;
    }

    private static Material LightMaterial()
    {
        if (sharedLightMaterial != null) return sharedLightMaterial;
        var shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        sharedLightMaterial = new Material(shader) { name = "CF_StatusLight (runtime)" };
        return sharedLightMaterial;
    }

    private void LateUpdate()
    {
        if (lightRenderer == null) return;

        SI.DroneState d = null;
        if (Telemetry != null) Telemetry.TryGetDrone(CfName, out d);
        Current = d;

        SetLight(LightColor(d));
        if (label == null) return;

        label.text = LabelText(d);
        label.color = d == null ? new Color(1f, 1f, 1f, 0.5f)
                    : DroneTelemetry.HasFault(d) ? DroneTelemetry.FaultCol
                    : DroneTelemetry.HasWarning(d) ? DroneTelemetry.WarnCol
                    : Color.white;

        // Billboard: keep the label upright and readable from the camera / headset.
        if (cam == null) cam = Camera.main;
        if (cam != null)
        {
            var away = labelT.position - cam.transform.position;
            if (away.sqrMagnitude > 1e-6f) labelT.rotation = Quaternion.LookRotation(away, Vector3.up);
        }
    }

    private Color LightColor(SI.DroneState d)
    {
        if (d == null) return DroneTelemetry.NoData;
        float t = Time.time;
        if (DroneTelemetry.HasFault(d))
            return Mathf.Repeat(t, 0.8f) < 0.4f ? DroneTelemetry.FaultCol : DroneTelemetry.FaultCol * 0.35f;
        var c = DroneTelemetry.StatusColor(DroneTelemetry.Status(d));
        if (DroneTelemetry.HasWarning(d))
            c = Color.Lerp(c, DroneTelemetry.WarnCol, 0.5f + 0.5f * Mathf.Sin(t * 6f));
        return c;
    }

    private string LabelText(SI.DroneState d)
    {
        if (d == null) return $"{CfName}\nno telemetry";
        string head = $"{CfName}  {Mathf.RoundToInt((float)d.battery_percent)}%";
        var faults = DroneTelemetry.FaultText(DroneTelemetry.Faults(d));
        if (faults.Length > 0) return $"{head}\n{faults}";
        var warnings = DroneTelemetry.WarningText(DroneTelemetry.Warnings(d));
        if (warnings.Length > 0) return $"{head}\n{warnings}";
        var status = DroneTelemetry.Status(d);
        string detail = status == DroneStatus.Searching || status == DroneStatus.Returning
            ? DroneTelemetry.Label(status)
            : DroneTelemetry.Label(DroneTelemetry.Mode(d));
        return $"{head}\n{detail}";
    }

    private void SetLight(Color c)
    {
        lightRenderer.GetPropertyBlock(block);
        block.SetColor(BaseColorId, c);
        block.SetColor(ColorId, c);
        lightRenderer.SetPropertyBlock(block);
    }
}
