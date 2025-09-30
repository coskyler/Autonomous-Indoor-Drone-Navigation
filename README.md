# Autonomous Indoor Drone Navigation
A Unity project simulating a LiDAR-mounted drone that autonomously explores and maps indoor environments, streaming scans to [Fleet Control Dashboard](https://fleetcontrol.coskyler.com/scans/7)

## Watch the Demo
[![Watch the demo](https://img.youtube.com/vi/1ApMZhoQgPc/hqdefault.jpg)](https://www.youtube.com/watch?v=1ApMZhoQgPc)

---

## Tech
- **Drone simulation:** physics-based motion with acceleration and deceleration
- **LiDAR simulation:** raycast sweeps generating configurable point clouds
- **Voxel mapping:** scalable grid-based occupancy stored in chunks of byte arrays
- **3D DDA:** voxel traversal algorithm used in line of sight checks and marking free cells along each ray
- **Frontier detection & goal management:** identifies unknown boundaries and manages exploration targets
- **Hybrid pathfinding:** weighted Theta* with jump point extension over voxels and A* over waypoints, followed by line-of-sight path smoothing
- **Dynamic waypoint graph:** line-of-sight connectivity with auto-bridging for scalability
- **GPU-instanced visualization:** efficient rendering of voxels, waypoints, frontiers, and paths

---

## Quick Setup
1. Clone or download this repository
2. Open the project in **Unity 6 (tested on 6000.0.0f1, URP)**
3. Open the small demo scene at `Assets/Scenes/SampleScene`
4. Open [the dashboard](https://fleetcontrol.coskyler.com/scans/7) in your browser (and create an account if you want to save your scans)
5. Run the unity scene, and paste the fleet code from the console into the dashboard
6. *(Optional)* Experiment with your own scenes and adjust the drone’s component variables (see tooltips)

*Note: To test without using the dashboard, enable `ScanStarted` and increase `MaxSpeed` (~2.5 recommended) in the Drone's components*
