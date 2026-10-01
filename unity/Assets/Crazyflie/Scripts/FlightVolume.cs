using UnityEngine;

// Draws the flight volume (wire box) and a floor grid with LineRenderers, so the operator
// can see the space the drones are allowed to use. Place it under the World root at (0,0,0):
// the box is centred on the ROS world origin in x/z and goes from the floor up to Size.y.
public class FlightVolume : MonoBehaviour
{
    [Tooltip("Unity axes: x = width, y = height, z = depth (metres).")]
    public Vector3 Size = new Vector3(6f, 3f, 6f);
    public float GridSpacing = 0.5f;
    public float LineWidth = 0.01f;
    public Color BoxColor = new Color(0.2f, 0.8f, 1f, 1f);
    public Color GridColor = new Color(1f, 1f, 1f, 0.25f);

    [Tooltip("Unlit/transparent material. If empty, one is created from Sprites/Default.")]
    public Material LineMaterial;

    private const string RootName = "__FlightVolumeLines";

    // Lines are built at runtime (visible in the headset); in edit mode the box is drawn as a gizmo.
    private void Start() => Rebuild();

    [ContextMenu("Rebuild (Play mode)")]
    public void Rebuild()
    {
        if (!Application.isPlaying) return;
        var old = transform.Find(RootName);
        if (old != null) Destroy(old.gameObject);

        var root = new GameObject(RootName).transform;
        root.SetParent(transform, false);
        var mat = LineMaterial != null ? LineMaterial : new Material(Shader.Find("Sprites/Default"));

        float hx = Size.x / 2f, hz = Size.z / 2f, h = Size.y;
        Vector3[] c =
        {
            new Vector3(-hx, 0, -hz), new Vector3(hx, 0, -hz), new Vector3(hx, 0, hz), new Vector3(-hx, 0, hz),
            new Vector3(-hx, h, -hz), new Vector3(hx, h, -hz), new Vector3(hx, h, hz), new Vector3(-hx, h, hz)
        };
        int[,] edges = { {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7} };
        for (int i = 0; i < edges.GetLength(0); i++)
            Line(root, mat, BoxColor, c[edges[i, 0]], c[edges[i, 1]]);

        if (GridSpacing > 0f)
        {
            for (float x = -hx + GridSpacing; x < hx - 1e-4f; x += GridSpacing)
                Line(root, mat, GridColor, new Vector3(x, 0.001f, -hz), new Vector3(x, 0.001f, hz));
            for (float z = -hz + GridSpacing; z < hz - 1e-4f; z += GridSpacing)
                Line(root, mat, GridColor, new Vector3(-hx, 0.001f, z), new Vector3(hx, 0.001f, z));
        }
    }

    private void Line(Transform parent, Material mat, Color color, Vector3 a, Vector3 b)
    {
        var go = new GameObject("line");
        go.transform.SetParent(parent, false);
        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = false;
        lr.positionCount = 2;
        lr.SetPosition(0, a);
        lr.SetPosition(1, b);
        lr.widthMultiplier = LineWidth;
        lr.sharedMaterial = mat;
        lr.startColor = lr.endColor = color;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
    }

    private void OnDrawGizmos()
    {
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = BoxColor;
        Gizmos.DrawWireCube(new Vector3(0, Size.y / 2f, 0), Size);
    }
}
