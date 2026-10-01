# One Crazyflie in Unity (Crazyswarm2 sim + ros-sharp)

Milestone 1 for the swarm: one simulated Crazyflie (`cf231`), flown by Crazyswarm2's
`crazyflie_sim` on ROS 2 Jazzy, shown and commanded from Unity. Unity does **no physics**
for the drone; the firmware-in-the-loop simulator owns the state, Unity visualizes it and
sends high-level commands.

```
ROS 2 Jazzy (Ubuntu 24.04)                              Unity 6 (this repo)
crazyflie_server (crazyflie_sim, backend:=sim)
  ├─ /tf   world -> cf231  ─────────────┐               ROS  [RosConnector + CrazyflieTfListener]
  ├─ /clock                             │  rosbridge    World
  └─ /cf231/{takeoff,land,go_to,        ├─ ws://:9090 ─  ├─ FlightVolume
             emergency}  ◄──────────────┘               ├─ cf231 [CrazyflieFollower + CrazyflieCommander]
                                                         └─ GoalMarker
```

---

## Part A — ROS 2 backend (Ubuntu 24.04 machine or VM)

### A1. ROS 2 Jazzy + packages
Install ROS 2 Jazzy (https://docs.ros.org/en/jazzy/Installation.html), then:

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-rosbridge-suite ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-motion-capture-tracking ros-jazzy-tf-transformations \
  libboost-program-options-dev libusb-1.0-0-dev swig python3-dev build-essential git
pip3 install --break-system-packages rowan cflib transforms3d   # Ubuntu 24.04 needs the flag
```

`~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash          # after A3
export PYTHONPATH=$HOME/crazyflie-firmware/build:$PYTHONPATH   # after A2
```

### A2. Crazyflie firmware Python bindings (the simulator runs the real firmware)
```bash
cd ~
git clone --branch 2025.02 --single-branch --recursive https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
make cf2_defconfig
make bindings_python
python3 -c "import cffirmware; print('firmware bindings OK')"   # with PYTHONPATH set
```

### A3. Crazyswarm2 workspace
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recursive https://github.com/IMRCLab/crazyswarm2
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### A4. One drone
`src/crazyswarm2/crazyflie/config/crazyflies.yaml` already has exactly one enabled drone:
`cf231`, `initial_position: [0.0, 0.0, 0.0]`. Leave it. In `config/server.yaml` keep
`sim.visualizations.rviz.enabled: true` — that plugin is what publishes the `/tf` pose Unity reads
(it is not the RViz GUI).

### A5. Run
```bash
# T1 — simulator (no RViz GUI, no joystick)
ros2 launch crazyflie launch.py backend:=sim rviz:=False teleop:=False

# T2 — rosbridge (same sourced env, so it knows crazyflie_interfaces)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# T3 — sanity checks
ros2 topic echo /tf --once                  # frame_id: world, child_frame_id: cf231
ros2 service list | grep cf231              # /cf231/takeoff, /cf231/land, /cf231/go_to ...
ros2 service call /cf231/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
```

---

## Part B — Unity (this project, Unity 6000.3, URP)

Scripts are in `Assets/Crazyflie/Scripts/`:

| Script | Put on | Job |
|---|---|---|
| `CrazyflieTfListener` | `ROS` object (with `RosConnector`) | One `/tf` subscription; latest pose per drone, ROS→Unity conversion |
| `CrazyflieFollower` | the drone | Moves the drone to its simulated pose (smoothed, stale flag) |
| `CrazyflieCommander` | the drone | `takeoff` / `land` / `go_to` / `emergency` service calls; T/L/G/E keys |
| `FlightVolume` | `World/FlightVolume` | Wire box + floor grid of the flight area |
| `Messages/CrazyflieInterfaces.cs` | — | C# versions of `crazyflie_interfaces` Takeoff/Land/GoTo |

### B1. Drone model
1. Download `crazyflie_description/urdf/cf2_assembly_with_props.dae` from the Crazyswarm2 repo
   (or `meshes/collada_files/cf2_assembly_with_props.dae` from `bitcraze/crazyflie-simulation`).
2. Drop it into `Assets/Crazyflie/Models/`. Unity imports Collada directly.
3. Create an empty GameObject `cf231`, drag the model under it and rename it `Visual`.
   The root carries the ROS pose; `Visual` only fixes the mesh. Check in the Scene view:
   the drone should be flat (propellers up) with its front along the root's blue (+Z) axis.
   If it is on its side, rotate `Visual` (typically X −90°); if the size is off, the real
   frame is ~9 cm motor-to-motor.
4. Add a material (URP/Lit) so it is not magenta.

### B2. Scene (`Assets/Scenes/CrazyflieSimulation.unity`)
```
ROS           RosConnector + CrazyflieTfListener
World         position 0,0,0 | rotation 0,0,0 | scale 1   (= ROS "world" frame)
 ├─ FlightVolume   FlightVolume (Size 6 x 3 x 6)
 ├─ cf231          CrazyflieFollower + CrazyflieCommander (CfName = cf231)
 │   └─ Visual     the model
 └─ GoalMarker     small sphere, e.g. local (1, 1, 1)  (no collider needed)
Ground        Plane, scale (3,1,3) at y = 0, dark URP/Lit material
Directional Light
Main Camera   e.g. position (0, 1.6, -4), looking at the origin
```
On the drone's `CrazyflieCommander`, drag `GoalMarker` into **Goal Marker**.

### B3. RosConnector settings
| Field | Value |
|---|---|
| Ros Bridge Server Url | `ws://<ubuntu-ip>:9090` (`hostname -I` on Ubuntu) |
| Protocol | WebSocketNET |
| Serializer | Newtonsoft JSON |
| ROS version | ROS2 |

Only one `RosConnector` in the scene. Do not add the TurtleBot prefab or `ClockPublisher`
(the simulator publishes `/clock`).

### B4. Run
1. Backend running (A5, T1 + T2).
2. Press **Play**. `cf231` snaps to (0,0,0) on the floor.
3. **T** take off → rises to 0.5 m. **G** fly to GoalMarker. **L** land. **E** emergency stop.
   (Same actions via right-click on the `CrazyflieCommander` component header.)
4. Optional cross-check: `ros2 run crazyflie_examples hello_world --ros-args -p use_sim_time:=True`
   should fly the Unity drone too.

---

## Coordinate convention
ROS world: x forward, y left, z up (right-handed). Unity: x right, y up, z forward (left-handed).
`unity = (-ros.y, ros.z, ros.x)`, quaternion `(ros.y, -ros.z, -ros.x, ros.w)`; the inverse is used for
`go_to` goals. ROS +x therefore shows as Unity +z.

## Troubleshooting
| Symptom | Check |
|---|---|
| Drone never moves | `ros2 topic echo /tf --once` shows `world → cf231`? rviz visualization enabled in `server.yaml`? Serializer = Newtonsoft? |
| "Not connected to rosbridge" | URL/IP, port 9090 open (`sudo ufw allow 9090/tcp`), VM network in bridged mode |
| Service call does nothing | rosbridge terminal must have `~/ros2_ws/install/setup.bash` sourced; watch it for errors |
| `ImportError: cffirmware` | `PYTHONPATH` missing the firmware `build` folder in the simulator terminal |
| Drone sideways / wrong size | fix the `Visual` child, never the `cf231` root |
| Motion looks fast-forwarded | the sim may run faster than real time; see `sim.max_dt` in `server.yaml` |
| Quest (Android) build fails on `BuiltinInterfaces` | add `ROS2` to Player Settings → Android → Scripting Define Symbols (currently only Standalone has it) |
