# XR-UAV-Swarm

**Control a simulated Crazyflie swarm from Unity — takeoff, land, go-to and formations — through a ROS 2 Jazzy + Crazyswarm2 backend.**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)
![Crazyswarm2](https://img.shields.io/badge/Crazyswarm2-sim-blue)
![Unity](https://img.shields.io/badge/Unity-XR-black?logo=unity)
![Docker](https://img.shields.io/badge/Docker-required-2496ED?logo=docker)

This is a self-contained multi-robot lab project: a swarm of six simulated
[Crazyflie](https://www.bitcraze.io/products/crazyflie-2-1/) quadrotors runs in
[Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) (the real Crazyflie firmware controller,
software-in-the-loop) inside a Docker container, and a Unity application visualises the swarm in 3D
and sends commands to it over [rosbridge](https://github.com/RobotWebTools/rosbridge_suite).

Between the two sits a small **swarm API** (`swarm_control`) that turns high-level requests,
such as *"fly a circle formation around (1, 0.5, 1)"*, into per-drone commands, and checks
them against a safety layer (geofence, minimum separation, speed limit) before anything moves.



<!-- Add a demo GIF here: ![demo](docs/media/demo.gif) -->

---

## Features

- **Simulated multi-drone swarm:** six Crazyflies fly in Crazyswarm2's simulator, which runs the real Crazyflie firmware controller in software, all inside a Docker container with ROS 2 Jazzy. No hardware and no ROS install on the host are needed.
- **Swarm control from Unity:** a 3D Unity scene connects over rosbridge, mirrors every drone live and sends commands to the whole swarm or to selected drones: takeoff, land, go-to, hover/stop.
- **Formations:** line, grid, circle and V shapes around any centre point, with adjustable spacing and heading. Each drone gets its own slot, and the swarm arrives at the formation together.
- **Safety and health monitoring:** every command is checked against a geofence, a minimum separation distance and a speed limit. `/swarm/state` streams position, battery, link quality, warnings and faults for each drone at 10 Hz, and drones with a fault are refused new commands.
- **Autonomous end-to-end test flight:** `smoke_test` flies the whole mission without anyone at the controls (takeoff → grid → line → land). Along the way it injects faults (low battery, weak link, motor failure), checks that the system reacts correctly, and prints PASS/FAIL for each check. It finishes in about a minute and always leaves the swarm landed.

---

## System architecture

```
┌──────────────────────────┐   WebSocket :9090   ┌──────────────────────────────────────────────┐
│  Unity XR app (unity/)   │ ◀─────────────────▶ │  ROS 2 Jazzy backend (Docker, ros2_ws/)      │
│                          │     (rosbridge)     │                                              │
│  • 3D swarm view         │                     │  rosbridge_server                            │
│  • drone selection       │  /swarm/* services  │        │                                     │
│  • command UI            │ ──────────────────▶ │  swarm_commander  (safety layer, formations) │
│                          │                     │        │  /cfN/takeoff|land|go_to            │
│                          │  /swarm/state, /tf  │        ▼                                     │
│                          │ ◀────────────────── │  Crazyswarm2 crazyflie_server (sim backend)  │
└──────────────────────────┘                     └──────────────────────────────────────────────┘
```

| Layer | Runs on | Tech |
|---|---|---|
| Visualisation + UI | Mac / Windows (Unity Editor) or XR headset | Unity, ROS# (ros-sharp) |
| Bridge | backend machine | rosbridge_suite (WebSocket, port 9090) |
| Swarm API | backend machine | `swarm_control` / `swarm_interfaces` (this repo) |
| Simulation | backend machine | Crazyswarm2 `sim` backend + Crazyflie firmware bindings |

**Command flow, briefly:** Unity calls a `/swarm/*` service → `swarm_commander` validates it against
the safety limits and computes per-drone targets → it calls each drone's Crazyswarm2 services
(`/cfN/takeoff`, `/cfN/go_to`, …) → the simulator flies the drones with the real firmware controller →
poses come back on `/tf` and aggregated state on `/swarm/state` → Unity updates the 3D view.

---

## Repository structure

## Repository structure

```
XR-UAV-Swarm/
├── README.md
├── docker/
│   └── Dockerfile                  # ROS 2 Jazzy + Crazyswarm2 + firmware bindings + rosbridge (versions pinned)
├── scripts/
│   ├── build_image.sh              # build the Docker image (once)
│   ├── create_container.sh         # create the dev container (once)
│   ├── swarm_up.sh                 # shortcut: start container, build, launch the bringup
│   └── shell.sh                    # open a terminal inside the container
├── ros2_ws/                        # ROS 2 workspace (mounted into the container)
│   └── src/
│       ├── swarm_interfaces/
│       │   ├── msg/                # DroneState, SwarmState, CommanderState, CommanderDrone
│       │   └── srv/                # SwarmCommand, GoTo, Formation, InjectFault, SetMissionStatus
│       └── swarm_control/
│           ├── launch/
│           │   └── swarm_bringup.launch.py   # sim + rosbridge + commander + telemetry
│           ├── config/
│           │   ├── crazyflies_6.yaml         # drone list + initial positions
│           │   ├── swarm_limits.yaml         # geofence + min separation (shared)
│           │   ├── swarm_commander.yaml      # speed, durations, formation spacing, health gating
│           │   └── swarm_telemetry.yaml      # battery, radio link, data freshness, motion thresholds
│           ├── swarm_control/
│           │   ├── swarm_commander.py        # node: /swarm/* command services + safety layer
│           │   ├── swarm_telemetry.py        # node: /swarm/state, fault injection, mission status
│           │   ├── smoke_test.py             # autonomous end-to-end test flight
│           │   ├── geometry.py               # formations, slot assignment, fence, separation (pure Python)
│           │   ├── flight_phase.py           # landed / taking off / flying / landing logic (pure Python)
│           │   └── telemetry_model.py        # battery, link, warnings, faults model (pure Python)
│           └── test/                         # pytest unit tests for the pure-Python modules
└── unity/                          # Unity project (open this folder in Unity Hub)
    ├── Assets/
    ├── Packages/
    └── ProjectSettings/
```

The ROS nodes are kept thin: all the logic lives in plain Python modules
(`geometry.py`, `flight_phase.py`, `telemetry_model.py`), so it can be unit-tested without ROS.

---

## Prerequisites

**Backend machine** (tested on Ubuntu 22.04 with an RTX 5070 Ti)
- Linux with [Docker Engine](https://docs.docker.com/engine/install/), with your user in the `docker` group
- Optional: NVIDIA GPU + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (only needed for RViz)
- No ROS installation on the host. ROS 2 Jazzy runs inside the container, so the host's Ubuntu version doesn't matter.

**Visualisation machine** (can be the same machine)
- Unity Hub + the Unity version listed in [`unity/ProjectSettings/ProjectVersion.txt`](unity/ProjectSettings/ProjectVersion.txt)
- [Git LFS](https://git-lfs.com) (Unity binary assets are stored with LFS)
- On the same network as the backend machine

---

## Installation

### 1. Clone

```bash
git lfs install                       # once per machine
git clone https://github.com/kavinaya2808/XR-UAV-Swarm.git
cd XR-UAV-Swarm
```

> **Backend-only machine?** Skip the Unity binaries:
> `GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/kavinaya2808/XR-UAV-Swarm.git`

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

## Running

**Terminal 1: bringup (Crazyswarm2 sim + rosbridge + swarm commander + telemetry)**

```bash
./scripts/shell.sh                    # enter the container
cd /root/ros2_ws
colcon build --symlink-install        # first time only
source install/setup.bash
ros2 launch swarm_control swarm_bringup.launch.py
```

Launch options: `rviz:=True` · `port:=9091` · `commander:=False` · `telemetry:=False`

> Shortcut: `./scripts/swarm_up.sh` does all of the above in one command from the host
> and prints the WebSocket URL for Unity.

**Unity:** set **RosConnector → Ros Bridge Server Url** to `ws://<backend-ip>:9090`, then press **Play**.
The six drones should appear on the ground.

**Terminal 2: autonomous test flight**

```bash
./scripts/shell.sh
source /root/ros2_ws/install/setup.bash

ros2 run swarm_control smoke_test                 # full run: flight + fault injection (~1 min)
ros2 run swarm_control smoke_test --skip-faults   # flight checks only
```

Exit code `0` means every check passed. Run it with Unity open to watch the swarm fly.

**Terminal 2: manual commands**

```bash
ros2 service call /swarm/takeoff   swarm_interfaces/srv/SwarmCommand "{}"
ros2 service call /swarm/formation swarm_interfaces/srv/Formation \
  "{shape: circle, center: {x: 1.0, y: 0.5, z: 1.0}}"
ros2 service call /swarm/go_to     swarm_interfaces/srv/GoTo \
  "{drone_ids: [cf1], targets: [{x: 0.5, y: -0.5, z: 1.0}]}"
ros2 topic echo /swarm/state --once
ros2 service call /swarm/land      swarm_interfaces/srv/SwarmCommand "{}"
```

**Unit tests** (pure Python, no ROS needed)

```bash
cd /root/ros2_ws/src/swarm_control
python3 -m pytest -q test/
```

**After changing ROS code**

```bash
./scripts/shell.sh
colcon build --symlink-install && source install/setup.bash
```

Python files and existing launch/config files are symlinked, so edits to them need no rebuild.
New files and `.msg` / `.srv` changes do need a rebuild.

---

## Formations

Formations are computed in `swarm_control/geometry.py` (pure Python, unit-tested in `test/`) and
placed around a centre point with configurable spacing and heading.

| Shape | Description |
|---|---|
| `line` | Drones in a straight row, perpendicular to the heading |
| `grid` | Rectangular grid, filled row by row |
| `circle` | Evenly spaced on a ring around the centre |
| `v` | V / chevron shape pointing along the heading |

Before any drone moves, every formation passes through the safety layer (geofence and minimum separation).
If the requested spacing is too tight for the swarm size, the service returns `success: false`
and a message explaining why.

---

## Swarm API (Unity ↔ ROS)

### Commands (`swarm_commander`)

| Service | Type | Description |
|---|---|---|
| `/swarm/takeoff` | `swarm_interfaces/srv/SwarmCommand` | Take off listed drones (empty = all), optional height and duration |
| `/swarm/land` | `swarm_interfaces/srv/SwarmCommand` | Land listed drones (empty = all) |
| `/swarm/go_to` | `swarm_interfaces/srv/GoTo` | One target per drone, or one shared offset for the group (`relative: true`) |
| `/swarm/formation` | `swarm_interfaces/srv/Formation` | `line`, `grid`, `circle`, `v` around a centre, with spacing + heading |
| `/swarm/stop` | `std_srvs/srv/Trigger` | All flying drones hover in place |

### State and testing (`swarm_telemetry`)

| Name | Type | Description |
|---|---|---|
| `/swarm/state` | `swarm_interfaces/msg/SwarmState` (10 Hz) | Per drone: position, velocity, battery, link quality, flight mode, status, target, nearest neighbour, warnings, faults + readable alerts. Plus a swarm summary (airborne / warnings / faults) |
| `/swarm/inject_fault` | `swarm_interfaces/srv/InjectFault` | Simulate problems: `battery`, `battery_drain`, `link_weak`, `link_loss`, `motor`, `sensor`, `position_loss`. `active: false` clears; `fault: all` clears everything |
| `/swarm/set_mission_status` | `swarm_interfaces/srv/SetMissionStatus` | Tag airborne drones as `searching`, `returning` or `none` so the UI can show what each one is doing |
| `/swarm/commander_state` | `swarm_interfaces/msg/CommanderState` | Internal commander state (flight phases, active targets), read by telemetry |
| `/tf` | `tf2_msgs/msg/TFMessage` | `world → cf1 … cfN` poses from the simulator |

Every service returns `success` + `message` (e.g. *"cf1 and cf2 would be 0.05 m apart"*) so the UI can show feedback.

**Warnings vs faults:** a *warning* means the drone can still fly but the operator should look
(low battery, weak link, too close to a neighbour, outside the fence, stale position).
A *fault* means it must not fly (battery depleted, link lost, motor/sensor failure, position lost).
A drone with a fault reports status `fault`, and the commander refuses new takeoff and move commands for it.

Example: simulate a motor failure on `cf2`, then clear it:

```bash
ros2 service call /swarm/inject_fault swarm_interfaces/srv/InjectFault "{drone_ids: [cf2], fault: motor}"
ros2 service call /swarm/inject_fault swarm_interfaces/srv/InjectFault "{fault: all, active: false}"
```

---

## Safety layer

Every command passes through these checks before any drone moves:

- **Flight volume:** x −1…3 m, y −1…2 m, z 0.2…2 m. Targets outside are clamped (`fence_mode: clamp`) or the command is refused (`fence_mode: reject`).
- **Minimum separation:** 0.3 m between any two planned positions.
- **Speed limit:** 0.5 m/s. Durations are stretched to respect it, and drones in one command arrive together (`sync_group: true`).
- **State check:** only flying drones accept go-to / formation commands.
- **Health gating:** drones with a fault, or with critically low battery, are refused takeoff and moves (`health_gating: true`).

The geofence and minimum separation live in **one shared file**, `swarm_limits.yaml`,
so the commander (which blocks commands) and telemetry (which raises warnings) always use the same numbers.

---

## Configuration

All files are in `ros2_ws/src/swarm_control/config/`:

| File | What to change there |
|---|---|
| `crazyflies_6.yaml` | Drone list and start positions (read by Crazyswarm2, commander and telemetry) |
| `swarm_limits.yaml` | Geofence and minimum separation |
| `swarm_commander.yaml` | Max speed, takeoff height, durations, default formation spacing, fence mode, health gating |
| `swarm_telemetry.yaml` | Simulated battery (endurance, drain, recharge, thresholds), radio link range, stale/lost timeouts |

Thanks to `--symlink-install`, config edits need no rebuild: just restart the bringup.
In `swarm_telemetry.yaml`, write numbers with a decimal point (`100.0`, not `100`), because ROS 2 rejects integers for float parameters.

**Change the swarm size:** copy `crazyflies_6.yaml` to `crazyflies_<N>.yaml`, edit the drone list, then launch with:

```bash
ros2 launch swarm_control swarm_bringup.launch.py \
  crazyflies_yaml_file:=/root/ros2_ws/src/swarm_control/config/crazyflies_<N>.yaml
```

**Faster testing:** set `drain_scale: 5.0` in `swarm_telemetry.yaml` to watch the battery warnings appear within a short flight.

---


## Acknowledgements

- [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) (IMRCLab): Crazyflie ROS 2 stack and simulator (MIT)
- [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware) (Bitcraze): controller used in simulation (GPL-3.0)
- [ROS#](https://github.com/siemens/ros-sharp) (Siemens): Unity ↔ rosbridge client (Apache-2.0; licence kept in its folder under `unity/Assets`)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite): WebSocket bridge between ROS 2 and Unity

---
