using System.Collections.Generic;
using System.Collections;
using System;
using UnityEngine;
using System.Diagnostics;

public class Main : MonoBehaviour
{
    [Header("LiDAR Settings")]
    [Tooltip("Horizontal rays per full 360° sweep")]
    public int horizontalResolution = 72;

    [Tooltip("Vertical rays per fov sweep")]
    public int verticalResolution = 18;

    [Tooltip("Vertical field of view in degrees")]
    public float verticalFOV = 90;

    [Tooltip("Maximum scan distance")]
    public float maxScanDistance = 30;

    [Tooltip("Maximum process distance. Must be less than max scan distance")]
    public float maxProcessDistance = 11; // Must be less than max scan distance

    [Tooltip("Layers to scan")]
    public LayerMask detectionLayers; // Raycast detection layers

    [Header("Map Settings")]
    public float voxelSize = 0.3f;
    public int clearanceRadius = 1;

    [Header("Visual Settings")]
    public bool renderFrontiers;
    public bool renderWaypoints;
    public bool renderEdges;
    public bool renderOccupied = true;
    public bool renderGoals;

    struct PointData
    {
        public Vector3 endPoint;
        public float rayLength;
        public bool hit;
    }

    struct FrontierGroup
    {
        public Vector3 centroid;
        public int size;
        public float incidenceAngle;

        public FrontierGroup(Vector3 centroid, int size, float incidenceAngle)
        {
            this.centroid = centroid;
            this.size = size;
            this.incidenceAngle = incidenceAngle;
        }
    }

    struct ExplorationGoal
    {
        public Vector3Int position;
        public int size;
        public bool reachable;
        public bool persist;

        public ExplorationGoal(Vector3Int position, int size, bool reachable, bool persist)
        {
            this.position = position;
            this.size = size;
            this.reachable = reachable;
            this.persist = persist;
        }
    }

    struct Edge
    {
        public Vector3Int to;
        public float w;

        public Edge(Vector3Int to, float w) { this.to = to; this.w = w; }
    }

    VoxelMap voxelMap;
    FlightController flightController;
    LineRenderer pathRenderer;

    Vector3 previousFrontierTarget;
    Vector3 startPosition;

    List<PointData> pointCloud = new();
    List<ExplorationGoal> previousGoals = new();
    List<Vector3Int> droneFlightPath;
    List<Vector3Int> sphereOffsets = new(); // Used to set clearance around occupied voxels
    
    HashSet<Vector3Int> unreachableVoxels = new();
    HashSet<Vector3Int> frontierVoxels = new();
    HashSet<Vector3Int> newFrees = new HashSet<Vector3Int>(); // Used to store potential frontiers

    readonly Vector3Int[] dirs = new Vector3Int[6];
    readonly Vector3Int[] semiDiagDirs = new Vector3Int[18];
    readonly Vector3Int[] diagDirs = new Vector3Int[26];


    Dictionary<int, int> occupiedPerLevel = new();
    Dictionary<int, int> freePerLevel = new();
    Dictionary<Vector3Int, List<Edge>> waypointGraph = new();

    Mesh sphereMesh;
    Mesh cubeMesh;
    Mesh cylinderMesh;
    Material cubeMat;
    Material debugMat;
    Material waypointMat;
    Material goalMat;

    bool pathfinderRunning = false; // Guard flag

    void Start()
    {
        voxelMap = new VoxelMap(Vector3.zero, 16, voxelSize);

        flightController = GetComponent<FlightController>();
        previousFrontierTarget = new Vector3(int.MaxValue, int.MaxValue, int.MaxValue); // Filler for the first frame
        droneFlightPath = new();
        startPosition = transform.position;
        flightController.setTarget(startPosition);

        // Initialize the meshes
        var tempCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cubeMesh = tempCube.GetComponent<MeshFilter>().sharedMesh;
        DestroyImmediate(tempCube);

        var tempSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphereMesh = tempSphere.GetComponent<MeshFilter>().sharedMesh;
        DestroyImmediate(tempSphere);

        var tempCylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cylinderMesh = tempCylinder.GetComponent<MeshFilter>().sharedMesh;
        DestroyImmediate(tempCylinder);


        // Create materials and enable GPU instancing
        cubeMat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        cubeMat.enableInstancing = true;
        cubeMat.color = Color.maroon;
        cubeMat.SetShaderPassEnabled("ShadowCaster", false);
        cubeMat.DisableKeyword("_RECEIVE_SHADOWS_OFF");
        cubeMat.EnableKeyword("_RECEIVE_SHADOWS_OFF");

        debugMat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        debugMat.enableInstancing = true;
        debugMat.color = Color.yellow;
        debugMat.SetShaderPassEnabled("ShadowCaster", false);
        debugMat.DisableKeyword("_RECEIVE_SHADOWS_OFF");
        debugMat.EnableKeyword("_RECEIVE_SHADOWS_OFF");

        waypointMat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        waypointMat.enableInstancing = true;
        waypointMat.color = Color.purple;
        waypointMat.SetShaderPassEnabled("ShadowCaster", false);
        waypointMat.DisableKeyword("_RECEIVE_SHADOWS_OFF");
        waypointMat.EnableKeyword("_RECEIVE_SHADOWS_OFF");

        goalMat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        goalMat.enableInstancing = true;
        goalMat.color = Color.blue;
        goalMat.SetShaderPassEnabled("ShadowCaster", false);
        goalMat.DisableKeyword("_RECEIVE_SHADOWS_OFF");
        goalMat.EnableKeyword("_RECEIVE_SHADOWS_OFF");



        // Initialiaze directional arrays
        int n1 = 0;
        int n2 = 0;
        int n3 = 0;
        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
                for (int k = -1; k <= 1; k++)
                {
                    int steps = 0;
                    if (i != 0) steps++;
                    if (j != 0) steps++;
                    if (k != 0) steps++;
                    if (steps == 0) continue;

                    if (steps <= 3)
                        diagDirs[n1++] = new Vector3Int(i, j, k);

                    if (steps <= 2)
                        semiDiagDirs[n2++] = new Vector3Int(i, j, k);

                    if (steps <= 1)
                        dirs[n3++] = new Vector3Int(i, j, k);
                }

        // Initialize clearance offsets
        HashSet<Vector3Int> sphereOffsetsSet = new();

        int radius = clearanceRadius;
        int rSqr = radius * radius;

        for (int x = -radius; x <= radius; x++)
            for (int y = -radius; y <= radius; y++)
                for (int z = -radius; z <= radius; z++)
                {
                    Vector3Int offset = new(x, y, z);
                    if (offset.sqrMagnitude <= rSqr)
                    {
                        sphereOffsetsSet.Add(offset);
                    }
                }

        foreach (Vector3Int v in sphereOffsetsSet)
        {
            sphereOffsets.Add(v);
        }

        // Initialize the line renderer
        GameObject lineObj = new GameObject("PathLine");
        pathRenderer = lineObj.AddComponent<LineRenderer>();

        pathRenderer.startWidth = 0.04f;
        pathRenderer.endWidth   = 0.04f;
        pathRenderer.positionCount = 0;
        pathRenderer.useWorldSpace = true;
        pathRenderer.loop = false;
        pathRenderer.numCapVertices = 4;
        pathRenderer.material = new Material(Shader.Find("Sprites/Default"));
        pathRenderer.startColor = Color.green;
        pathRenderer.endColor   = Color.green;


    }

    float pathfindCooldown = 0.5f;
    float pathfindCooldownTimer = 1.5f;
    float addWpCd = 0.5f;
    float addWpCdTimer = 1.5f;

    void Update()
    {
        float dt = Time.deltaTime;
        pathfindCooldownTimer -= dt;
        addWpCdTimer -= dt;

        Stopwatch diagnosticsTimer = Stopwatch.StartNew();
        String diagnostics = $"{1f / dt} fps";

        PerformScan();
        diagnostics += $"\nScan time: {diagnosticsTimer.ElapsedMilliseconds}";

        ProcessPointCloud();
        diagnostics += $"\nVoxel processing: {diagnosticsTimer.ElapsedMilliseconds}";
        
        List<FrontierGroup> frontierGroups = updateFrontiers();
        diagnostics += $"\nFrontier processing: {diagnosticsTimer.ElapsedMilliseconds}";

        if (!pathfinderRunning && pathfindCooldownTimer < 0)
        {
            StartCoroutine(navigate(frontierGroups));
            pathfindCooldownTimer = pathfindCooldown;
        }

        // Create new waypoint
        if (addWpCdTimer < 0)
        {
            addWaypoint(toVoxelPos(transform.position), false);
            addWpCdTimer = addWpCd;

            diagnostics += $"\nCreating waypoint: {diagnosticsTimer.ElapsedMilliseconds}";
        }

        followPath();

        // Visualize waypoint graph
        if (renderWaypoints)
        {
            List<Matrix4x4> waypointMatrices = new();
            List<Matrix4x4> edgeMatrices = new();
            foreach (var kvp in waypointGraph)
            {
                waypointMatrices.Add(Matrix4x4.TRS(toWorldPos(kvp.Key), Quaternion.identity, Vector3.one * voxelSize * .5f));

                if (renderEdges)
                    foreach (Edge edge in kvp.Value)
                        edgeMatrices.Add(LineMatrix(toWorldPos(kvp.Key), toWorldPos(edge.to), .025f));
            }

            // Render waypoints
            for (int i = 0; i < waypointMatrices.Count; i += 1023)
            {
                int batchSize = Mathf.Min(1023, waypointMatrices.Count - i);
                Graphics.DrawMeshInstanced(sphereMesh, 0, waypointMat, waypointMatrices.GetRange(i, batchSize));
            }

            // Render edges
            if (renderEdges)
                for (int i = 0; i < edgeMatrices.Count; i += 1023)
                {
                    int batchSize = Mathf.Min(1023, edgeMatrices.Count - i);
                    Graphics.DrawMeshInstanced(cylinderMesh, 0, waypointMat, edgeMatrices.GetRange(i, batchSize));
                }
        }

        // Visualize frontiers
        if (renderFrontiers)
        {
            List<Matrix4x4> frontierMatrices = new();
            foreach (Vector3Int frontier in frontierVoxels)
            {
                frontierMatrices.Add(Matrix4x4.TRS((Vector3)frontier * voxelSize + Vector3.one * (voxelSize * 0.5f), Quaternion.identity, Vector3.one * voxelSize * .75f));
            }
            for (int i = 0; i < frontierMatrices.Count; i += 1023)
            {
                int batchSize = Mathf.Min(1023, frontierMatrices.Count - i);
                Graphics.DrawMeshInstanced(cubeMesh, 0, debugMat, frontierMatrices.GetRange(i, batchSize));
            }
        }

        // Visualize exploration goals
        if (renderGoals)
        {
            List<Matrix4x4> goalMatrices = new();
            foreach (ExplorationGoal goal in previousGoals)
            {
                goalMatrices.Add(Matrix4x4.TRS(toWorldPos(goal.position), Quaternion.identity, Vector3.one * MathF.Cbrt(goal.size) * voxelSize / 2));
            }
            for (int i = 0; i < goalMatrices.Count; i += 1023)
            {
                int batchSize = Mathf.Min(1023, goalMatrices.Count - i);
                Graphics.DrawMeshInstanced(sphereMesh, 0, goalMat, goalMatrices.GetRange(i, batchSize));
            }
        }

        if (renderOccupied)
            voxelMap.Render(cubeMesh, cubeMat);
    
        
        UnityEngine.Debug.Log(diagnostics);
    }

    void PerformScan()
    {
        pointCloud.Clear();
        Vector3 origin = transform.position;

        float verticalStep = verticalFOV / (verticalResolution - 1);
        float verticalStart = -verticalFOV / 2f + verticalStep / 2;

        for (int v = 0; v < verticalResolution - 1; v++)
        {
            float basePitch = verticalStart + v * verticalStep;

            for (int h = 0; h < horizontalResolution; h++)
            {
                float pitch = basePitch + UnityEngine.Random.Range(-verticalStep / 2, verticalStep / 2);

                float horizontalStep = 360f / horizontalResolution;
                float yaw = horizontalStep * h + UnityEngine.Random.Range(-horizontalStep / 2, horizontalStep / 2);
                Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);

                Vector3 direction = rotation * Vector3.forward;

                if (Physics.Raycast(origin, direction, out RaycastHit hit, maxScanDistance, detectionLayers, QueryTriggerInteraction.Ignore))
                {
                    if (hit.distance > maxProcessDistance)
                    {
                        pointCloud.Add(new PointData { endPoint = origin + direction * maxProcessDistance, hit = true, rayLength = hit.distance });
                    }
                    else
                    {
                        pointCloud.Add(new PointData { endPoint = hit.point, hit = true, rayLength = hit.distance });
                    }
                    
                }
                else
                {
                    pointCloud.Add(new PointData { endPoint = origin + direction * maxProcessDistance, hit = false, rayLength = hit.distance });
                }
            }
        }
    }

    void ProcessPointCloud()
    {
        foreach (PointData point in pointCloud)
        {
            // Convert point to voxel
            Vector3Int voxel = new Vector3Int(
                Mathf.FloorToInt(point.endPoint.x / voxelSize),
                Mathf.FloorToInt(point.endPoint.y / voxelSize),
                Mathf.FloorToInt(point.endPoint.z / voxelSize)
            );

            bool inProcessRange = point.rayLength <= maxProcessDistance;

            // Add voxel at point to voxel map
            int hitState = voxelMap.Get(voxel);
            if (hitState != 1)
            {
                if (point.hit && inProcessRange)
                {
                    if (hitState == 2)
                    {
                        freePerLevel[voxel.y] = Mathf.Max(0, freePerLevel.GetValueOrDefault(voxel.y, 0) - 1);
                    }
                    voxelMap.Add(voxel, 1);
                    occupiedPerLevel[voxel.y] = occupiedPerLevel.GetValueOrDefault(voxel.y, 0) + 1;
                    frontierVoxels.Remove(voxel);

                    // Mark nearby voxels as no-clearance
                    foreach (Vector3Int offset in sphereOffsets)
                    {
                        voxelMap.SetClearance(voxel + offset, false);
                    }
                }
                else
                {
                    if (hitState != 2)
                    {
                        voxelMap.Add(voxel, 2);
                        freePerLevel[voxel.y] = freePerLevel.GetValueOrDefault(voxel.y, 0) + 1;
                    }
                    if (point.hit)
                        newFrees.Add(voxel);
                }
            }
        }

        // Mark all voxels between hit and drone as free
        Stopwatch stopwatch = Stopwatch.StartNew();
        foreach (PointData point in pointCloud) TraverseFrees(point.endPoint, point.hit); //Store all free voxels between drone pos and point
    }

    
    List<FrontierGroup> updateFrontiers()
    {
        Vector3 pos = transform.position;
        Vector3Int voxelPos = toVoxelPos(pos);

        List<ExplorationGoal> currentGoals = new();

        HashSet<Vector3Int> unreachableGoals = new();

        // Persist previous goals to current goals if conditions met
        for (int i = 0; i < previousGoals.Count; i++)
        {
            ExplorationGoal goal = previousGoals[i];
            var nearestV = findNearestTraversableVoxel(goal.position);
            var dist = Vector3.Distance(toWorldPos(goal.position), pos);
            if (goal.persist && nearestV.HasValue)
                if (dist >= maxProcessDistance * 0.8 || (!LineOfSight(nearestV.Value, voxelPos, true)) && dist >= 3)
                {
                    currentGoals.Add(goal);
                    if (unreachableVoxels.Contains(goal.position)) goal.reachable = false;
                }
            if (!goal.reachable)
                unreachableGoals.Add(goal.position);
        }

        foreach (var v in unreachableVoxels)
        {
            unreachableGoals.Add(v);
        }

        unreachableVoxels.Clear();

        // If frontier is valid, check if its group is within range
        List<Vector3Int> toRemove = new();
        HashSet<Vector3Int> visited = new();
        foreach (Vector3Int frontier in frontierVoxels)
        {
            if (Vector3.Distance(toWorldPos(frontier), pos) > maxProcessDistance * 1.5)
            {
                if (visited.Contains(frontier)) continue;

                // Floodfill frontier groups
                int floodSize = 0;
                Vector3 floodSum = Vector3.zero;

                Queue<Vector3Int> ff = new();
                HashSet<Vector3Int> tempVisited = new();
                List<Vector3Int> currentGroup = new();

                ff.Enqueue(frontier);
                tempVisited.Add(frontier);

                bool frontierInRange = false;
                while (ff.Count != 0 && !frontierInRange)
                {
                    Vector3Int cur = ff.Dequeue();
                    floodSize++;
                    floodSum += toWorldPos(cur);
                    currentGroup.Add(cur);

                    foreach (Vector3Int dir in diagDirs)
                    {
                        Vector3Int next = cur + dir;
                        if (!frontierVoxels.Contains(next)) continue;
                        if (!tempVisited.Add(next)) continue;

                        if (visited.Contains(next) || Vector3.Distance(toWorldPos(next), pos) <= maxProcessDistance * 1.5)
                        {
                            frontierInRange = true;
                            currentGroup.Clear();
                            break;
                        }
                        ff.Enqueue(next);
                    }
                }

                foreach (Vector3Int t in tempVisited)
                    visited.Add(t);

                // Remove frontier groups that are outside range while keeping the exploration goal
                if (!frontierInRange)
                {
                    bool hadGoal = false;
                    Vector3Int centroid = toVoxelPos(floodSum / floodSize);
                    bool goalReachable = true;
                    foreach (ExplorationGoal goal in previousGoals)
                    {
                        if (goal.position == centroid)
                        {
                            hadGoal = true;
                            goalReachable = goal.reachable;
                        }
                    }

                    if (hadGoal)
                    {
                        currentGoals.Add(new ExplorationGoal(centroid, floodSize, goalReachable, true));
                        UnityEngine.Debug.Log("Adding persisting goal!");
                    }

                    foreach (Vector3Int v in currentGroup)
                    {
                        toRemove.Add(v);
                    }
                }
            }
        }
        
        foreach (Vector3Int voxel in toRemove)
        {
            frontierVoxels.Remove(voxel);
        }

        // Remove old invalid and far away frontiers
        toRemove.Clear();
        foreach (Vector3Int frontier in frontierVoxels)
        {
            bool isFrontier = false;
            foreach (Vector3Int dir in dirs)
            {
                Vector3Int neighbor = frontier + dir;
                if (voxelMap.Get(neighbor) == 0)
                {
                    isFrontier = true;
                    break;
                }
            }
            if (!isFrontier)
            {
                toRemove.Add(frontier);
            }
        }
        foreach (Vector3Int voxel in toRemove)
        {
            frontierVoxels.Remove(voxel);
        }


        // Mark frontier voxels from new free voxels
        foreach (Vector3Int free in newFrees)
        {
            Vector3 voxelCenter = toWorldPos(free);

            float distance = Vector3.Distance(pos, voxelCenter);
            Vector3 lookAt = (voxelCenter - pos).normalized;
            float pitch = Mathf.Abs(Mathf.Asin(lookAt.y)) * Mathf.Rad2Deg; // 0-90 degrees
            if (pitch > verticalFOV / 2 * .95) continue;

            foreach (Vector3Int dir in dirs)
            {
                Vector3Int neighbor = free + dir;
                if (voxelMap.Get(neighbor) == 0)
                {
                    frontierVoxels.Add(free);
                    break;
                }
            }
        }

        // Loop over frontiers
        HashSet<Vector3Int> seen = new();
        List<FrontierGroup> frontierGroups = new();

        foreach (Vector3Int frontier in frontierVoxels)
        {
            if (seen.Contains(frontier)) continue;
            // Floodfill frontier groups
            int floodSize = 0;
            int floodFloor = int.MaxValue;
            int floodCeiling = int.MinValue;
            int minFloodx = int.MaxValue;
            int maxFloodx = int.MinValue;
            int minFloodz = int.MaxValue;
            int maxFloodz = int.MinValue;
            Vector3 floodSum = Vector3.zero;
            Queue<Vector3Int> ff = new();
            ff.Enqueue(frontier);
            seen.Add(frontier);
            while (ff.Count != 0)
            {
                Vector3Int cur = ff.Dequeue();
                floodSize++;
                floodSum += toWorldPos(cur);
                if (cur.y > floodCeiling) floodCeiling = cur.y;
                if (cur.y < floodFloor) floodFloor = cur.y;

                if (cur.x > maxFloodx) maxFloodx = cur.x;
                if (cur.x < minFloodx) minFloodx = cur.x;

                if (cur.z > maxFloodz) maxFloodz = cur.z;
                if (cur.z < minFloodz) minFloodz = cur.z;


                foreach (Vector3Int dir in diagDirs)
                {
                    Vector3Int next = cur + dir;
                    if (seen.Contains(next)) continue;
                    if (!frontierVoxels.Contains(next)) continue;

                    seen.Add(next);
                    ff.Enqueue(next);
                }


            }
            // Filter frontier group
            int floodHeight = floodCeiling - floodFloor + 1;
            if (floodSize * voxelSize < 7) continue; // num of frontiers in group
            if (floodHeight * voxelSize < 1) continue; // height of frontier group

            // Very wide tall groups are likely outdoors
            if ((maxFloodx - minFloodx + maxFloodz - minFloodz) * voxelSize > maxProcessDistance * 3.5 || floodHeight * voxelSize > maxProcessDistance * .5)
                continue;

            Vector3 centriod = floodSum / floodSize;
            Vector3Int vCentroid = toVoxelPos(centriod);

            ExplorationGoal newGoal = new ExplorationGoal(vCentroid, floodSize, true, false);
            if (unreachableGoals.Contains(vCentroid)) newGoal.reachable = false;

            currentGoals.Add(newGoal);
        }

        // Convert goals to frontier groups (legacy)
        foreach (ExplorationGoal goal in currentGoals) {
            if (!goal.reachable) continue;
            FrontierGroup group = new FrontierGroup(toWorldPos(goal.position), goal.size, 0);
            frontierGroups.Add(group);
        }

        // Sort exploration goals heuristically
        Vector3 adjustedPos = flightController.getAdjustedPos(1);
        frontierGroups.Sort((a, b) =>
        {
            // size - adjusted dist - incidence angle - exponential height diff
            float aScore = (0f * a.size) - (10f * Vector3.Distance(adjustedPos, a.centroid)) - (0f * a.incidenceAngle) - Mathf.Pow(1f * Mathf.Abs(a.centroid.y - adjustedPos.y), 5);
            float bScore = (0f * b.size) - (10f * Vector3.Distance(adjustedPos, b.centroid)) - (0f * b.incidenceAngle) - Mathf.Pow(1f * Mathf.Abs(b.centroid.y - adjustedPos.y), 5);
            return bScore.CompareTo(aScore); // Highest score first
        });

        newFrees.Clear();
        previousGoals = currentGoals;
        return frontierGroups;
    }

    IEnumerator navigate(List<FrontierGroup> frontierGroups)
    {
        pathfinderRunning = true;

        Vector3 pos = transform.position;
        Vector3Int currentVoxel = toVoxelPos(pos);


        List<Vector3Int> path = new();

        // Frontier groups move as they are explored. If a nearby (shifted) frontier group is present with LOS, remain on path
        FrontierGroup nearestGroup = new FrontierGroup();
        float nearestGroupDist = float.PositiveInfinity;
        foreach (FrontierGroup group in frontierGroups)
        {
            float distToGroup = Vector3.Distance(group.centroid, previousFrontierTarget);
            if (distToGroup < nearestGroupDist)
            {
                nearestGroup = group;
                nearestGroupDist = distToGroup;
            }
        }
        Vector3Int? correctedTarget = findNearestTraversableVoxel(toVoxelPos(nearestGroup.centroid));
        if (correctedTarget.HasValue && Vector3.Distance(nearestGroup.centroid, previousFrontierTarget) < 2)
        {
            path = pathfind(currentVoxel, correctedTarget.Value);
        }

        if (path.Count > 0 && !(Vector3.Distance(nearestGroup.centroid, pos) > maxProcessDistance * 2 && thetaStarPathfind(currentVoxel, toVoxelPos(frontierGroups[0].centroid), 20).Count > 0))
        {
            previousFrontierTarget = nearestGroup.centroid;
            droneFlightPath = path;
        }
        else
        {
            unreachableVoxels.Add(toVoxelPos(nearestGroup.centroid));
            for (int i = 0; i < frontierGroups.Count; i++)
            {
                Vector3Int? ptargetFrontierGroupPos = findNearestTraversableVoxel(Vector3Int.FloorToInt(frontierGroups[i].centroid / voxelSize));
                Vector3Int targetFrontierGroupPos;

                if (ptargetFrontierGroupPos.HasValue)
                {
                    targetFrontierGroupPos = ptargetFrontierGroupPos.Value;
                }
                else
                {
                    continue;
                }

                path = pathfind(currentVoxel, targetFrontierGroupPos);
                if (path.Count > 0)
                {
                    previousFrontierTarget = toWorldPos(targetFrontierGroupPos);
                    droneFlightPath = path;
                    break;
                }
                else
                {
                    unreachableVoxels.Add(targetFrontierGroupPos);
                }
            }
        }

        pathfinderRunning = false;
        yield return null;
    }

    void followPath()
    {
        // Replace start of path
        if (droneFlightPath.Count > 1)
        {
            List<Vector3Int> pathStart = new();
            int newStart = 0;
            for (int i = 1; i < droneFlightPath.Count; i++)
            {
                List<Vector3Int> potentialStart = thetaStarPathfind(toVoxelPos(transform.position), droneFlightPath[i], 10);
                if (potentialStart.Count != 0)
                {
                    pathStart = potentialStart;
                    newStart = i + 1;
                }
                else
                {
                    break;
                }
            }

            if (pathStart.Count > 0)
            {
                pathStart.RemoveAt(0);
                droneFlightPath.RemoveRange(0, newStart);
                pathStart.AddRange(droneFlightPath);
                droneFlightPath = pathStart;
            }
        }

        // Remove waypoint(s) that we're currently on
        while (droneFlightPath.Count > 0 && Vector3.Distance(toWorldPos(droneFlightPath[0]), transform.position) < voxelSize)
        {
            droneFlightPath.RemoveAt(0);
        }

        // Set target to next waypoint
        if (droneFlightPath.Count > 0)
        {
            flightController.setTarget(toWorldPos(droneFlightPath[0]));
        }

        List<Vector3> toRender = new();
        toRender.Add(transform.position);
        foreach (Vector3Int pathPoint in droneFlightPath)
            toRender.Add(toWorldPos(pathPoint));

        DrawPathLines(toRender);
    }

    Vector3Int? findNearestTraversableVoxel(Vector3Int voxel)
    {
        Queue<Vector3Int> queue = new();
        HashSet<Vector3Int> visited = new();

        queue.Enqueue(voxel);
        visited.Add(voxel);

        while (queue.Count > 0)
        {
            Vector3Int cur = queue.Dequeue();

            if (ManhattanDistance(voxel, cur) > 4) break;

            bool traversable = voxelMap.CanTraverse(cur);
            if (traversable) return cur;

            foreach (Vector3Int dir in dirs)
            {
                Vector3Int next = cur + dir;
                if (!visited.Add(next)) continue;
                queue.Enqueue(next);
            }
        }

        return null;
    }

    void TraverseFrees(Vector3 target, bool hit)
    {

        Vector3 origin = transform.position;
        Vector3 dir = (target - origin).normalized;

        Vector3Int currentVoxel = Vector3Int.FloorToInt(origin / voxelSize);
        Vector3Int targetVoxel = Vector3Int.FloorToInt(target / voxelSize);

        Vector3Int step = new Vector3Int(
            dir.x >= 0 ? 1 : -1,
            dir.y >= 0 ? 1 : -1,
            dir.z >= 0 ? 1 : -1
        );

        Vector3Int originVoxel = Vector3Int.FloorToInt(origin / voxelSize);
        Vector3 voxelBoundary = new Vector3(
            (originVoxel.x + (step.x > 0 ? 1 : 0)) * voxelSize,
            (originVoxel.y + (step.y > 0 ? 1 : 0)) * voxelSize,
            (originVoxel.z + (step.z > 0 ? 1 : 0)) * voxelSize
        );

        Vector3 tMax = new Vector3(
            dir.x == 0 ? float.PositiveInfinity : Mathf.Abs((voxelBoundary.x - origin.x) / dir.x),
            dir.y == 0 ? float.PositiveInfinity : Mathf.Abs((voxelBoundary.y - origin.y) / dir.y),
            dir.z == 0 ? float.PositiveInfinity : Mathf.Abs((voxelBoundary.z - origin.z) / dir.z)
        );

        Vector3 tDelta = new Vector3(
            dir.x == 0 ? float.PositiveInfinity : Mathf.Abs(voxelSize / dir.x),
            dir.y == 0 ? float.PositiveInfinity : Mathf.Abs(voxelSize / dir.y),
            dir.z == 0 ? float.PositiveInfinity : Mathf.Abs(voxelSize / dir.z)
        );

        // Soft cap: should never hit this unless something goes wrong
        for (int i = 0; i < 1000; i++)
        {
            if (currentVoxel == targetVoxel)
                break;

            float dist = Vector3.Distance(transform.position, (Vector3)currentVoxel * voxelSize + Vector3.one * voxelSize / 2f);
            if (voxelMap.Get(currentVoxel) != 1)
            {
                if (voxelMap.Get(currentVoxel) != 2)
                {
                    voxelMap.Add(currentVoxel, 2);
                    freePerLevel[currentVoxel.y] = freePerLevel.GetValueOrDefault(currentVoxel.y, 0) + 1;
                }
                if (hit)
                    newFrees.Add(currentVoxel);
            }

            if (tMax.x < tMax.y && tMax.x < tMax.z)
            {
                currentVoxel.x += step.x;
                tMax.x += tDelta.x;
            }
            else if (tMax.y < tMax.z)
            {
                currentVoxel.y += step.y;
                tMax.y += tDelta.y;
            }
            else
            {
                currentVoxel.z += step.z;
                tMax.z += tDelta.z;
            }
        }
    }

    List<Vector3Int> pathfind(Vector3Int start, Vector3Int target)
    {
        List<Vector3Int> path;

        // Lightweight check if path can be found over pure voxel traversal
        path = thetaStarPathfind(start, target, 200);
        if (path.Count > 0) return path;

        // 1. Find nearest waypoint from current pos via BFS
        Vector3Int startWp = new Vector3Int(int.MaxValue, 0, 0);
        Dictionary<Vector3Int, Vector3Int> startPrevMap = new();

        { // BFS
            Queue<Vector3Int> queue = new();
            HashSet<Vector3Int> visited = new();

            queue.Enqueue(start);
            visited.Add(start);

            int n = 0;

            while (queue.Count > 0)
            {
                Vector3Int cur = queue.Dequeue();

                if (++n > 10000)
                {
                    UnityEngine.Debug.Log("Pathfinding error: no reachable waypoints found from current position. BFS exhausted.");
                    return new();
                }

                if (waypointGraph.ContainsKey(cur))
                {
                    startWp = cur;
                    break;
                }

                foreach (Vector3Int dir in dirs)
                {
                    Vector3Int next = cur + dir;
                    if (!voxelMap.CanTraverse(next) || !visited.Add(next)) continue;
                    queue.Enqueue(next);
                    startPrevMap[next] = cur;
                }
            }
        }

        if (startWp.x == int.MaxValue)
        {
            UnityEngine.Debug.Log("Pathfinding error: no reachable waypoints exist from current position.");
            return new();
        }

        // 2. Find closest waypoint to target. Try quick pass with theta* else BFS from target
        Vector3Int endWp = new Vector3Int(int.MaxValue, 0, 0);
        Dictionary<Vector3Int, Vector3Int> endPrevMap = new();

        Vector3Int closestWayPoint = Vector3Int.zero;
        float closestDist = float.PositiveInfinity;
        foreach (Vector3Int wp in waypointGraph.Keys)
        {
            float dist = EuclideanDistance(wp, target);
            if (dist < closestDist)
            {
                closestDist = dist;
                closestWayPoint = wp;
            }
        }

        List<Vector3Int> toTarget = thetaStarPathfind(closestWayPoint, target, 100);

        // If path cannot be found via closest waypoint, BFS
        if (toTarget.Count > 0)
        {
            endWp = closestWayPoint;
        }
        else {
            Queue<Vector3Int> queue = new();
            HashSet<Vector3Int> visited = new();

            queue.Enqueue(target);
            visited.Add(target);

            int n = 0;

            while (queue.Count > 0)
            {
                Vector3Int cur = queue.Dequeue();

                if (++n > 10000)
                {
                    UnityEngine.Debug.Log("Pathfinding error: no reachable waypoints found from target position. BFS exhausted.");
                    return new();
                }

                if (waypointGraph.ContainsKey(cur))
                {
                    endWp = cur;
                    break;
                }

                foreach (Vector3Int dir in dirs)
                {
                    Vector3Int next = cur + dir;
                    if (!voxelMap.CanTraverse(next) || !visited.Add(next)) continue;
                    queue.Enqueue(next);
                    endPrevMap[next] = cur;
                }
            }
        }

        if (endWp.x == int.MaxValue)
        {
            UnityEngine.Debug.Log("Pathfinding error: no reachable waypoints exist from target position.");
            return new();
        }

        bool foundPath = false;
        Dictionary<Vector3Int, Vector3Int> wpPrevMap = new();

        // 3. Pathfind between waypoints with A*
        {
            PriorityQueue<Vector3Int, float> pqueue = new();
            Dictionary<Vector3Int, float> weights = new();
            HashSet<Vector3Int> closed = new();

            pqueue.Enqueue(startWp, EuclideanDistance(startWp, endWp));
            weights[startWp] = 0;

            while (pqueue.Count > 0)
            {
                Vector3Int cur = pqueue.Dequeue();
                if (!closed.Add(cur)) continue;

                float gScore = weights[cur];

                if (cur == endWp)
                {
                    foundPath = true;
                    break;
                }

                foreach (Edge edge in waypointGraph[cur])
                {
                    if (closed.Contains(edge.to)) continue;

                    float weight = gScore + edge.w;
                    if (weights.GetValueOrDefault(edge.to, float.PositiveInfinity) > weight)
                    {
                        pqueue.Enqueue(edge.to, weight + EuclideanDistance(edge.to, endWp));
                        weights[edge.to] = weight;
                        wpPrevMap[edge.to] = cur;
                    }
                }
            }
        }

        if (!foundPath)
        {
            UnityEngine.Debug.Log("Pathfinding error: no path exists between waypoints.");
            return new();
        }

        // 4. Reconstruct path
        {
            Vector3Int cur;

            // Start -> start waypoint
            cur = startWp;
            List<Vector3Int> toStartWp = new();
            while (startPrevMap.TryGetValue(cur, out Vector3Int cameFrom))
            {
                cur = cameFrom;
                toStartWp.Add(cur);
            }

            toStartWp.Reverse();

            // Start waypoint -> end waypoint
            cur = endWp;
            List<Vector3Int> toEndWp = new();
            while (wpPrevMap.TryGetValue(cur, out Vector3Int cameFrom))
            {
                cur = cameFrom;
                toEndWp.Add(cur);
            }

            toEndWp.Reverse();

            // End waypoint -> target
            if (toTarget.Count == 0)
            {
                cur = endWp;
                toTarget.Add(cur);
                while (endPrevMap.TryGetValue(cur, out Vector3Int cameFrom))
                {
                    cur = cameFrom;
                    toTarget.Add(cur);
                }
            }

            path.AddRange(toStartWp);
            path.AddRange(toEndWp);
            path.AddRange(toTarget);
        }

        // 5. Smooth path
        return smoothPath(path);
    }

    List<Vector3Int> smoothPath(List<Vector3Int> path)
    {
        if (path.Count <= 2) return path;

        List<Vector3Int> smoothedPath = new();
        smoothedPath.Add(path[0]);

        Vector3Int anchor = path[0];
        for(int i = 2; i < path.Count; i++)
        {
            if (!LineOfSight(anchor, path[i], true))
            {
                smoothedPath.Add(path[i - 1]);
                anchor = path[i - 1];
            }
        }

        if (path[path.Count - 1] != smoothedPath[smoothedPath.Count - 1])
            smoothedPath.Add(path[path.Count - 1]);


        return smoothedPath;
    }

    List<Vector3Int> thetaStarPathfind(Vector3Int start, Vector3Int target, int explorationLimit)
    {
        int cellsExplored = 0;
        Stopwatch stopwatch = Stopwatch.StartNew();

        Dictionary<Vector3Int, Vector3Int> predecessorMap = new();
        Dictionary<Vector3Int, float> gScores = new();
        HashSet<Vector3Int> losTraversed = new();
        PriorityQueue<Vector3Int, float> queue = new();
        float w = 2; // Heuristic weight (prioritizes search towards target)
        queue.Enqueue(start, EuclideanDistance(start, target));
        gScores[start] = 0;
        predecessorMap[start] = start;

        while (queue.Count != 0)
        {
            Vector3Int pos = queue.Dequeue();

            float gScore = gScores[pos];
            float hScore = EuclideanDistance(pos, target);
            Vector3Int cameFrom = predecessorMap[pos];

            cellsExplored++;

            Vector3Int jumpPoint = findJumpPoint(pos, target, losTraversed);

            bool nearFrontier = EuclideanDistance(pos, target) <= clearanceRadius * voxelSize * 1.5 && LineOfSight(pos, target, false);
            if (jumpPoint == target) nearFrontier = true;
            if (nearFrontier || cellsExplored > explorationLimit)
            {
                stopwatch.Stop();
                if (!nearFrontier)
                {
                    //UnityEngine.Debug.Log($"No path found. Theta* Elapsed time: {stopwatch.ElapsedMilliseconds} ms, {cellsExplored} cells explored, los voxel count: {losTraversed.Count}");
                    return new();
                }

                //UnityEngine.Debug.Log($"Theta* Elapsed time: {stopwatch.ElapsedMilliseconds} ms, {cellsExplored} cells explored, los voxel count: {losTraversed.Count}");

                List<Vector3Int> path = new();
                Vector3Int cur = pos;
                if (jumpPoint == target) path.Add(jumpPoint);
                while (cur != start)
                {
                    path.Add(cur);
                    cur = predecessorMap[cur];
                }
                path.Add(start);
                path.Reverse();
                return path;
            }

            if (jumpPoint != pos && voxelMap.CanTraverse(jumpPoint))
            {
                float jpGScore = gScores[pos] + EuclideanDistance(pos, jumpPoint);
                float jpHScore = EuclideanDistance(jumpPoint, target);

                float prevJpGScore = gScores.TryGetValue(jumpPoint, out float prevJpG) ? prevJpG : float.PositiveInfinity;

                if (jpGScore < prevJpGScore)
                {
                    queue.Enqueue(jumpPoint, jpGScore + w * jpHScore);
                    gScores[jumpPoint] = jpGScore + 5;
                    predecessorMap[jumpPoint] = pos;
                }
            }

            foreach (Vector3Int dir in diagDirs)
            {
                Vector3Int next = pos + dir;
                if (!voxelMap.CanTraverse(next)) continue;
                bool touchingWall = false;
                foreach (Vector3Int d in diagDirs)
                    if (!voxelMap.CanTraverse(next + d) && voxelMap.Get(next + d) != 0)
                    {
                        touchingWall = true;
                        break;
                    }

                if (!touchingWall) continue;
                float nextHScore = EuclideanDistance(next, target);
                float nextGScore = gScores[cameFrom] + EuclideanDistance(cameFrom, next);
                bool los = true;
                float prevGScore = gScores.TryGetValue(next, out float prevG) ? prevG : float.PositiveInfinity;

                if (prevGScore <= nextGScore) continue;

                if (!LineOfSight(cameFrom, next, true))
                {
                    los = false;
                    nextGScore = gScore + EuclideanDistance(pos, next) + 5;
                }

                if (prevGScore <= nextGScore) continue;

                if (los)
                {
                    predecessorMap[next] = cameFrom;
                }
                else
                {
                    predecessorMap[next] = pos;
                }

                gScores[next] = nextGScore;
                queue.Enqueue(next, nextGScore + w * nextHScore);
            }
        }

        return new();
    }

    bool LineOfSight(Vector3Int start, Vector3Int target, bool sweepBound)
    {
        Vector3Int v = start;
        Vector3Int d = target - start;

        // Step direction per axis
        int stepX = (int)Mathf.Sign(d.x);
        int stepY = (int)Mathf.Sign(d.y);
        int stepZ = (int)Mathf.Sign(d.z);

        float tDeltaX = d.x == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(d.x);
        float tDeltaY = d.y == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(d.y);
        float tDeltaZ = d.z == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(d.z);

        float tMaxX = (d.x == 0) ? float.PositiveInfinity : 0.5f * tDeltaX;
        float tMaxY = (d.y == 0) ? float.PositiveInfinity : 0.5f * tDeltaY;
        float tMaxZ = (d.z == 0) ? float.PositiveInfinity : 0.5f * tDeltaZ;

        // Iterate until we reach the target voxel
        while (v != target)
        {
            if (tMaxX < tMaxY)
            {
                if (tMaxX < tMaxZ)
                {
                    v.x += stepX;
                    tMaxX += tDeltaX;
                }
                else
                {
                    v.z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }
            else
            {
                if (tMaxY < tMaxZ)
                {
                    v.y += stepY;
                    tMaxY += tDeltaY;
                }
                else
                {
                    v.z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }

            if (voxelMap.Get(v) == 1 || (sweepBound && !voxelMap.CanTraverse(v))) return false;

        }
        return true;
    }

    Vector3Int findJumpPoint(Vector3Int start, Vector3Int target, HashSet<Vector3Int> traversed)
    {
        Vector3Int v = start;
        Vector3Int d = target - start;

        // Step direction per axis
        int stepX = (int)Mathf.Sign(d.x);
        int stepY = (int)Mathf.Sign(d.y);
        int stepZ = (int)Mathf.Sign(d.z);

        float tDeltaX = d.x == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(d.x);
        float tDeltaY = d.y == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(d.y);
        float tDeltaZ = d.z == 0 ? float.PositiveInfinity : 1f / Mathf.Abs(d.z);

        float tMaxX = (d.x == 0) ? float.PositiveInfinity : 0.5f * tDeltaX;
        float tMaxY = (d.y == 0) ? float.PositiveInfinity : 0.5f * tDeltaY;
        float tMaxZ = (d.z == 0) ? float.PositiveInfinity : 0.5f * tDeltaZ;

        Vector3Int prevV = v;

        // Iterate until we reach the target voxel
        while (v != target)
        {
            if (tMaxX < tMaxY)
            {
                if (tMaxX < tMaxZ)
                {
                    v.x += stepX;
                    tMaxX += tDeltaX;
                }
                else
                {
                    v.z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }
            else
            {
                if (tMaxY < tMaxZ)
                {
                    v.y += stepY;
                    tMaxY += tDeltaY;
                }
                else
                {
                    v.z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }

            if (!traversed.Add(v)) return start;
            if (!voxelMap.CanTraverse(v)) return prevV;
            prevV = v;

        }
        return v;
    }

    void addWaypoint(Vector3Int voxel, bool bridge)
    {
        bool isNewVertex = !waypointGraph.ContainsKey(voxel);
        List<Vector3Int> keys = new(waypointGraph.Keys);

        List<Vector3Int> nearbyWaypoints = new();

        // Quick pass to check if new waypoint is too close to another
        if (isNewVertex && !bridge)
            foreach (Vector3Int waypoint in keys)
                if (Vector3.Distance(toWorldPos(voxel), toWorldPos(waypoint)) < 2) nearbyWaypoints.Add(waypoint);

        // If nearby waypoints exist, update their edges instead of creating a new waypoint
        if (nearbyWaypoints.Count == 0)
            nearbyWaypoints.Add(voxel);

        // Update each nearby waypoint (or create new one)
        foreach (Vector3Int v in nearbyWaypoints)
        {
            // Create edge list
            List<Edge> edges = new();
            List<Vector3Int> allWaypoints = new();
            foreach (Vector3Int waypoint in keys)
            {
                if (waypoint == v) continue;
                allWaypoints.Add(waypoint);
                if (LineOfSight(v, waypoint, true))
                {
                    edges.Add(new Edge(waypoint, EuclideanDistance(v, waypoint)));
                }
            }

            allWaypoints.Sort((a, b) =>
                EuclideanDistance(v, a).CompareTo(EuclideanDistance(v, b))
            );


            // Remove old reciprocal edges
            if (!isNewVertex)
                foreach (Edge edge in waypointGraph[v])
                {
                    List<Edge> oppEdges = waypointGraph[edge.to];
                    for (int i = oppEdges.Count - 1; i >= 0; i--)
                    {
                        if (oppEdges[i].to == v)
                        {
                            oppEdges[i] = oppEdges[oppEdges.Count - 1];
                            oppEdges.RemoveAt(oppEdges.Count - 1);
                            break;
                        }
                    }
                }

            // Add reciprocal edges
            foreach (Edge edge in edges)
                waypointGraph[edge.to].Add(new Edge(v, edge.w));

            waypointGraph[v] = edges;

            // If updated waypoint has no edges, create bridge edges to nearest waypoint
            if (edges.Count == 0 && !bridge)
            {
                List<Vector3Int> path = new();
                int i = -1;
                while (path.Count == 0 && ++i < Math.Min(3, allWaypoints.Count))
                {
                    if (allWaypoints[i] == v) continue;
                    path = thetaStarPathfind(v, allWaypoints[i], 200);
                }
                foreach (Vector3Int newBridge in path)
                    addWaypoint(newBridge, true);
                if (path.Count == 0) UnityEngine.Debug.Log("Critical error: Failed to bridge waypoints");
                else
                {
                    // Pass through bridges again to ensure connectivity to main graph
                    createEdge(path[0], v);
                    for (int j = 0; j < path.Count - 1; j++)
                        createEdge(path[j], path[j + 1]);
                    createEdge(path[path.Count - 1], allWaypoints[i]);
                }
            }
        }
    }

    void createEdge(Vector3Int wp, Vector3Int to)
    {

        List<Edge> edges = waypointGraph[wp];

        // Check if edge already exists
        foreach (Edge edge in edges)
        {
            if (edge.to == to) return;
        }

        // Add reciprocal and create edge
        waypointGraph[to].Add(new Edge(wp, EuclideanDistance(wp, to)));
        waypointGraph[wp].Add(new Edge(to, EuclideanDistance(wp, to)));
    }

    float EuclideanDistance(Vector3Int a, Vector3Int b) // For voxel space
    {
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        int dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dy * dy + dz * dz) * voxelSize;
    }
    
    int ManhattanDistance(Vector3Int a, Vector3Int b)
    {
        return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y) + Mathf.Abs(a.z - b.z);
    }


    Vector3Int toVoxelPos(Vector3 voxel)
    {
        return Vector3Int.FloorToInt(voxel / voxelSize);
    }

    Vector3 toWorldPos(Vector3Int voxel)
    {
        return (Vector3)voxel * voxelSize + Vector3.one * voxelSize / 2f;
    }

    void DrawPathLines(List<Vector3> path)
    {
        if (path == null || path.Count < 2)
        {
            pathRenderer.positionCount = 0;
            return;
        }

        pathRenderer.positionCount = path.Count;
        for (int i = 0; i < path.Count; i++)
        {
            Vector3 p = path[i];
            pathRenderer.SetPosition(i, p);
        }
    }

    Matrix4x4 LineMatrix(Vector3 start, Vector3 end, float thickness)
    {
        Vector3 mid = (start + end) * 0.5f;             // midpoint
        Vector3 dir = (end - start).normalized;         // direction
        float length = Vector3.Distance(start, end);    // length

        Quaternion rot = Quaternion.FromToRotation(Vector3.up, dir);

        Vector3 scale = new Vector3(thickness, length * 0.5f, thickness);

        return Matrix4x4.TRS(mid, rot, scale);
    }
}