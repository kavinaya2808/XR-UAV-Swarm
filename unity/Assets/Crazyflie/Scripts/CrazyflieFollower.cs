using UnityEngine;

// Moves this GameObject to the simulated Crazyflie's pose. The simulation (crazyflie_sim)
// owns the physics; Unity only visualizes. Keep the drone as a child of a "World" root whose
// origin is the ROS "world" frame, so the whole arena can later be moved or scaled as one.
public class CrazyflieFollower : MonoBehaviour
{
    [Tooltip("Name from crazyflies.yaml, e.g. cf231.")]
    public string CfName = "cf231";

    [Tooltip("Found automatically if left empty.")]
    public CrazyflieTfListener Listener;

    [Tooltip("Exponential smoothing rate. 0 = snap to every sample.")]
    [Range(0f, 40f)] public float Smoothing = 20f;

    [Tooltip("Pose older than this is reported as stale (link lost / sim stopped).")]
    public float StaleAfterSeconds = 1f;

    public bool HasPose { get; private set; }
    public bool IsStale { get; private set; } = true;

    private void Start()
    {
        if (Listener == null) Listener = FindFirstObjectByType<CrazyflieTfListener>();
        if (Listener == null) Debug.LogError("[Crazyflie] No CrazyflieTfListener in the scene.", this);
    }

    private void LateUpdate()
    {
        if (Listener == null) return;
        if (!Listener.TryGetPose(CfName, out var pos, out var rot, out var age))
        {
            IsStale = true;
            return;
        }

        if (!HasPose || Smoothing <= 0f)
        {
            transform.localPosition = pos;
            transform.localRotation = rot;
            HasPose = true;
        }
        else
        {
            float k = 1f - Mathf.Exp(-Smoothing * Time.deltaTime);
            transform.localPosition = Vector3.Lerp(transform.localPosition, pos, k);
            transform.localRotation = Quaternion.Slerp(transform.localRotation, rot, k);
        }
        IsStale = age > StaleAfterSeconds;
    }
}
