# XR-UAV-Swarm

**Mixed Reality interface for controlling and monitoring a UAV swarm.**
MSc Computer Science thesis, University of Bern (2026).

A Unity XR application visualises a swarm of simulated Crazyflie quadrotors and lets the
operator command them (takeoff, land, go-to, formations) through a ROS 2 backend. The
backend runs [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) in simulation (the real
Crazyflie firmware controller as software-in-the-loop) inside Docker, and exposes a small
swarm API to Unity over rosbridge (WebSocket).

---

## System architecture

```
┌──────────────────────────┐   WebSocket :9090   ┌──────────────────────────────────────────────┐
│  Unity XR app (unity/)   │ ◀─────────────────▶ │  ROS 2 Jazzy backend (Docker, ros2_ws/)      │
│                          │     (rosbridge)     │                                              │
│  • 3D swarm view         │                     │  rosbridge_server                            │
│  • drone selection       │  /swarm/* services  │        │                                     │
│  • MR command UI         │ ──────────────────▶ │  swarm_commander  (safety layer, formations) │
│                          │                     │        │  /cfN/takeoff|land|go_to            │
│                          │  /swarm/state, /tf  │        ▼                                     │
│                          │ ◀────────────────── │  Crazyswarm2 crazyflie_server (sim backend)  │
└──────────────────────────┘                     └──────────────────────────────────────────────┘
```

| Layer | Runs on | Tech |
|---|---|---|
| XR app | Mac / Windows (Unity Editor) or XR headset | Unity, ROS# (ros-sharp) |
| Bridge | backend machine | rosbridge_suite (WebSocket, port 9090) |
| Swarm API | backend machine | `swarm_control` / `swarm_interfaces` (this repo) |
| Simulation | backend machine | Crazyswarm2 `sim` backend + Crazyflie firmware bindings |

---

## Repository structure

```
XR-UAV-Swarm/
├── README.md
├── docker/
│   └── Dockerfile               # ROS 2 Jazzy + Crazyswarm2 + firmware bindings + rosbridge (versions pinned)
├── scripts/
│   ├── build_image.sh           # build the Docker image (once)
│   ├── create_container.sh      # create the dev container (once)
│   ├── swarm_up.sh              # ONE command: start container, build, launch everything
│   └── shell.sh                 # extra terminal inside the container
├── ros2_ws/                     # ROS 2 workspace (mounted into the container)
│   └── src/
│       ├── swarm_interfaces/    # msg: DroneState, SwarmState · srv: GoTo, Formation, SwarmCommand
│       └── swarm_control/
│           ├── launch/swarm_bringup.launch.py
│           ├── config/crazyflies_6.yaml       # drone list + initial positions
│           ├── config/swarm_commander.yaml    # safety limits, speeds, defaults
│           ├── swarm_control/swarm_commander.py
│           ├── swarm_control/geometry.py      # formations + safety maths (pure Python)
│           └── test/
├── unity/                       # Unity project (open this folder in Unity Hub)
│   ├── Assets/
│   ├── Packages/
│   └── ProjectSettings/
└── docs/
    └── dailylogs/               # development log
```

---

## Prerequisites

**Backend machine** (tested: Ubuntu 22.04, RTX 5070 Ti)
- Linux with [Docker Engine](https://docs.docker.com/engine/install/) (user in the `docker` group)
- Optional: NVIDIA GPU + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (only needed for RViz)
- Nothing ROS-related is installed on the host — ROS 2 Jazzy runs inside the container, so the host OS version doesn't matter

**XR machine**
- Unity Hub + the Unity version in [`unity/ProjectSettings/ProjectVersion.txt`](unity/ProjectSettings/ProjectVersion.txt)
- [Git LFS](https://git-lfs.com) (binary assets are stored with LFS)
- Same network as the backend machine

---

## Installation

### 1. Clone
```bash
git lfs install                       # once per machine
git clone https://github.com/<your-user>/XR-UAV-Swarm.git
cd XR-UAV-Swarm
```
> Backend-only machine? Skip the Unity binaries with
> `GIT_LFS_SKIP_SMUDGE=1 git clone ...`

### 2. Backend (Linux)
```bash
./scripts/build_image.sh              # ~15–25 min the first time
./scripts/create_container.sh         # creates container "xr-swarm", mounts ros2_ws/
```

### 3. Unity
1. Unity Hub → **Add project from disk** → select the `unity/` folder
2. Open it with the Unity version from `ProjectVersion.txt` (packages restore on first open)
3. Open the main scene in `Assets/Scenes/`

---

## Running a session

**Terminal 1 — backend (sim + rosbridge + swarm commander)**
```bash
./scripts/swarm_up.sh                 # prints the URL for Unity, e.g. ws://192.168.1.20:9090
# options: ./scripts/swarm_up.sh rviz:=True   |   port:=9091   |   commander:=False
```
First run builds `ros2_ws` automatically.

**Unity** — set the **RosConnector → Ros Bridge Server Url** to the printed `ws://<backend-ip>:9090`, then press Play.

**Terminal 2 — commands / checks** (optional)
```bash
./scripts/shell.sh
ros2 service call /swarm/takeoff   swarm_interfaces/srv/SwarmCommand "{}"
ros2 service call /swarm/formation swarm_interfaces/srv/Formation "{shape: circle, center: {x: 1.0, y: 0.5, z: 1.0}}"
ros2 service call /swarm/go_to     swarm_interfaces/srv/GoTo "{drone_ids: [cf1], targets: [{x: 0.5, y: -0.5, z: 1.0}]}"
ros2 topic echo /swarm/state --once
ros2 service call /swarm/land      swarm_interfaces/srv/SwarmCommand "{}"
```

**After changing ROS code**
```bash
./scripts/shell.sh
colcon build --symlink-install && source install/setup.bash
```
(Python files and existing launch/config files are symlinked — no rebuild needed for edits to those. New files and `.msg/.srv` changes need a rebuild.)

---

## Swarm API (Unity ↔ ROS)

| Name | Type | Description |
|---|---|---|
| `/swarm/takeoff` | `swarm_interfaces/srv/SwarmCommand` | Take off listed drones (empty = all) |
| `/swarm/land` | `swarm_interfaces/srv/SwarmCommand` | Land listed drones (empty = all) |
| `/swarm/go_to` | `swarm_interfaces/srv/GoTo` | One target per drone, or one shared offset (`relative: true`) |
| `/swarm/formation` | `swarm_interfaces/srv/Formation` | `line`, `grid`, `circle`, `v` around a centre, with spacing + heading |
| `/swarm/stop` | `std_srvs/srv/Trigger` | All flying drones hover in place |
| `/swarm/state` | `swarm_interfaces/msg/SwarmState` (10 Hz) | Position, status, target, nearest neighbour per drone + separation warning |
| `/tf` | `tf2_msgs/msg/TFMessage` | `world → cf1 … cfN` poses from the simulator |

Every service returns `success` + `message` (e.g. *"cf1 and cf2 would be 0.05 m apart"*) so the UI can show feedback.

**Safety layer** (`ros2_ws/src/swarm_control/config/swarm_commander.yaml`)
- Flight volume: x −1…3 m, y −1…2 m, z 0.2…2 m — targets outside are clamped (or rejected with `fence_mode: reject`)
- Minimum separation between planned positions: 0.3 m
- Speed limit: 0.5 m/s; drones in one command arrive together
- Only flying drones accept go-to / formation commands

**Changing the swarm size:** add a new `config/crazyflies_<N>.yaml` (copy `crazyflies_6.yaml`), then
`./scripts/swarm_up.sh crazyflies_yaml_file:=/root/ros2_ws/src/swarm_control/config/crazyflies_<N>.yaml`.

---

## Pinned versions

| Component | Version |
|---|---|
| ROS 2 | Jazzy (`osrf/ros:jazzy-desktop`, Ubuntu 24.04 in the container) |
| Crazyswarm2 | `f1e09954f56cd08e7d9aff5e13dc9a68795914a3` |
| Crazyflie firmware | see `CF_FIRMWARE_REF` in `docker/Dockerfile` |
| numpy | `< 2` (required by the sim backend) |
| Unity | see `unity/ProjectSettings/ProjectVersion.txt` |

To update Crazyswarm2 or the firmware: change the `ARG` in `docker/Dockerfile`, then
`./scripts/build_image.sh && docker rm -f xr-swarm && ./scripts/create_container.sh`.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `permission denied ... docker.sock` | `sudo usermod -aG docker $USER`, then log out/in (or `newgrp docker`) |
| `ros2: command not found` | You are on the host — use `./scripts/shell.sh` first |
| `Package 'swarm_control' not found` | `colcon build --symlink-install && source install/setup.bash` inside the container |
| rosbridge `[Errno 98] Address already in use` | rosbridge already runs inside `swarm_up.sh` — don't start a second one |
| `LaunchConfigurationEquals ... deprecated` warning | Comes from Crazyswarm2's launch file; harmless |
| Unity can't connect | Backend running? Correct IP in RosConnector? Same network? Firewall allows port 9090? |
| RViz: `could not connect to display` | `xhost +local:docker` on the host |
| Unity textures/models look broken after clone | Git LFS not installed: `git lfs install && git lfs pull` |
| Start completely fresh | `docker rm -f xr-swarm && ./scripts/create_container.sh` (your code in `ros2_ws/src` is untouched) |

---

## Acknowledgements

- [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) (IMRCLab) — Crazyflie ROS 2 stack and simulator (MIT)
- [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware) (Bitcraze) — controller used in simulation (GPL-3.0)
- [ROS#](https://github.com/siemens/ros-sharp) (Siemens) — Unity ↔ rosbridge client (Apache-2.0; licence kept in its folder under `unity/Assets`)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

## Author

Kavinaya — MSc Computer Science, University of Bern
