using UnityEngine;

public class FollowCam : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0, 0.5f, -1.5f);
    public float smoothSpeed = 3f;

    void LateUpdate()
    {
        if (!target) return;

        // Extract target yaw only
        Vector3 euler = target.rotation.eulerAngles;
        Quaternion yawOnly = Quaternion.Euler(0f, euler.y, 0f);

        // Apply yaw to the offset
        Vector3 desiredPos = target.position + yawOnly * offset;

        // Smooth follow
        transform.position = Vector3.Lerp(transform.position, desiredPos, smoothSpeed * Time.deltaTime);

        // Look at target
        transform.LookAt(target);
    }
}
