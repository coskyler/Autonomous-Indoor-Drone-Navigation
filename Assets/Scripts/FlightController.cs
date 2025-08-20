using UnityEngine;
public class FlightController : MonoBehaviour
{
    public bool active = true;
    public Vector3 target;

    [Tooltip("Speed cap (m/s)")]
    public float maxSpeed = 2.5f;

    [Tooltip("Acceleration cap (m/s^2)")]
    public float acceleration = 6;
    public float rotateSpeed = 360;
    public float maxTilt = 30;

    Vector3 velocity;

    public void setTarget(Vector3 target)
    {
        this.target = target;
    }

    public Vector3 getAdjustedPos(float multiplier)
    {
        return transform.position + velocity * multiplier;
    }

    void Update()
    {
        if (!active) return;

        Vector3 toTarget = target - transform.position;
        float distance = toTarget.magnitude;


        // Desired velocity from rest
        Vector3 dir = toTarget.normalized;
        float desiredSpeed = Mathf.Min(maxSpeed, Mathf.Sqrt(2f * acceleration * distance)); // v^2 = 2ad
        Vector3 desiredVel = dir * desiredSpeed;

        // Correct current velocity toward desired
        Vector3 delta = desiredVel - velocity;
        float maxDeltaMag = acceleration * Time.deltaTime;

        float accelDir = 1;
        if (velocity.magnitude > desiredVel.magnitude) accelDir = -1;

        if (delta.magnitude < maxDeltaMag)
        {
            velocity = desiredVel; // Can hit target velocity this frame
        }
        else
        {
            velocity += delta.normalized * maxDeltaMag; // Move toward it as fast as allowed
        }

        // Clamp velocity to desired speed
        if (velocity.magnitude > desiredSpeed)
            velocity = velocity.normalized * desiredSpeed;

        // Clamp velocity to maxSpeed
        if (velocity.magnitude > maxSpeed)
            velocity = velocity.normalized * maxSpeed;

        // Move and rotate
        Quaternion targetRotation = transform.rotation;

        float travelDist = velocity.magnitude * Time.deltaTime;
        if (travelDist >= distance || distance < 0.01)
        {
            transform.position = target;
            velocity = Vector3.zero;
        }
        else
        {
            transform.position += velocity * Time.deltaTime;
            if (velocity.x != 0 || velocity.z != 0)
                targetRotation = OrientToTarget(transform.rotation, Quaternion.LookRotation(target - transform.position));
        }

        Vector3 groundPlaneVlocity = new Vector3(velocity.x, 0f, velocity.z);
        float tiltDeg = Mathf.Clamp01(groundPlaneVlocity.magnitude * 4 / maxSpeed) * maxTilt * accelDir;

        Vector3 f = targetRotation * Vector3.forward;
        f.y = 0f;
        Quaternion yawOnly = f.sqrMagnitude < 1e-6f ? targetRotation : Quaternion.LookRotation(f.normalized, Vector3.up);
        targetRotation = Quaternion.AngleAxis(tiltDeg, yawOnly * Vector3.right) * yawOnly;


        Quaternion roll = Quaternion.AngleAxis(Mathf.Clamp(YawDifference(transform.rotation, targetRotation) * -10, -45, 45), transform.forward);
        targetRotation = roll * targetRotation;

        transform.rotation = SmoothRotate(transform.rotation, targetRotation, rotateSpeed);
    }


    public static Quaternion SmoothRotate(Quaternion current, Quaternion target, float degreesPerSecond)
    {
        float maxDegreesThisFrame = degreesPerSecond * Time.deltaTime;
        return Quaternion.RotateTowards(current, target, maxDegreesThisFrame);
    }

    public static Quaternion OrientToTarget(Quaternion current, Quaternion target)
    {
        Vector3 forward = target * Vector3.forward;
        forward.y = 0f;
        return Quaternion.LookRotation(forward.normalized, Vector3.up);
    }

    float YawDifference(Quaternion a, Quaternion b)
    {
        // Extract world yaw from both
        float yawA = Mathf.Atan2(2f * (a.y * a.w + a.x * a.z),
                                1f - 2f * (a.y * a.y + a.x * a.x)) * Mathf.Rad2Deg;

        float yawB = Mathf.Atan2(2f * (b.y * b.w + b.x * b.z),
                                1f - 2f * (b.y * b.y + b.x * b.x)) * Mathf.Rad2Deg;

        // Difference, wrapped to [-180, 180]
        float diff = Mathf.DeltaAngle(yawA, yawB);
        return diff;
    }
}
