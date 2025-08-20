# AIDN – Autonomous Indoor Drone Navigation
Autonomous Indoor Drone Navigation (AIDN) — a Unity project simulating a LiDAR-mounted drone that autonomously explores and maps indoor environments.

[![Watch the demo](https://img.youtube.com/vi/1ApMZhoQgPc/hqdefault.jpg)](https://www.youtube.com/watch?v=1ApMZhoQgPc)

---

## Tech
- **Drone simulation** – physics-based motion with acceleration and deceleration
- **LiDAR simulation** – raycast sweeps generating configurable point clouds
- **Voxel mapping** – scalable grid-based occupancy stored in chunks of byte arrays
- **3D DDA** – voxel traversal algorithm used in line of sight checks and marking free cells along each ray
- **Frontier detection & goal management** – identifies unknown boundaries and manages exploration targets
- **Hybrid pathfinding** – weighted Theta* with jump point extension over voxels and A* over waypoints, followed by line-of-sight path smoothing
- **Dynamic waypoint graph** – line-of-sight connectivity with auto-bridging for scalability
- **GPU-instanced visualization** – efficient rendering of voxels, waypoints, frontiers, and paths

---

## Quick Setup
1. Clone or download this repository
2. Open the project in **Unity (URP-compatible version, e.g. 2021 LTS or newer)**
3. That's it, a small demo scene is already set up
4. (Optional) Experiment with your own scenes and the component variables in the drone object (see tooltips)
