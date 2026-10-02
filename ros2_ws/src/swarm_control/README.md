# swarm_control

ROS 2 (Jazzy) layer between the Unity MR UI (rosbridge) and Crazyswarm2.

```
Unity ──ws:9090──▶ rosbridge ──▶ swarm_commander ──▶ Crazyswarm2 /cfN/takeoff|land|go_to ──▶ sim / Crazyflies
                                   │      ▲                         │
                                   │      └──── /swarm/state ◀──────┼── swarm_telemetry ◀── /tf
                                   └──── /swarm/commander_state ────┘          │
Unity, search, detection ◀──────────────────── /swarm/state (10 Hz) ───────────┘
```

## Nodes
| Node | File | Role |
|---|---|---|
| `swarm_commander` | `swarm_control/swarm_commander.py` | Services `/swarm/takeoff`, `land`, `go_to`, `formation`, `stop`. Safety layer (fence, separation, speed) + health gating |
| `swarm_telemetry` | `swarm_control/swarm_telemetry.py` | Publishes `/swarm/state` (`SwarmState`, one `DroneState` per drone). Services `/swarm/inject_fault`, `/swarm/set_mission_status` |
| `smoke_test` | `swarm_control/smoke_test.py` | End-to-end check against the running bringup (sim only) |

## Pure-Python logic (no ROS, unit-tested)
| Module | Contents |
|---|---|
| `geometry.py` | formation shapes, slot assignment, fence, separation, durations |
| `flight_phase.py` | landed / taking_off / flying / landing from time **and** measured height |
| `telemetry_model.py` | battery, link, flight mode, status, warnings, faults, fault injection |

## Config (`config/`)
| File | Used by |
|---|---|
| `crazyflies_6.yaml` | drone list — Crazyswarm2, commander and telemetry all read it |
| `swarm_limits.yaml` | fence + min separation, shared by commander and telemetry (`/**`) |
| `swarm_commander.yaml` | speed, durations, health gating |
| `swarm_telemetry.yaml` | battery, link, freshness, motion thresholds (write `100.0`, not `100`) |

## Run / test
```bash
ros2 launch swarm_control swarm_bringup.launch.py     # or: swarm_up (host)
ros2 run swarm_control smoke_test                      # ~1 min, needs the bringup
python3 -m pytest -q test/                             # unit tests, no ROS needed
```

## Conventions
- Every service returns `success` + `message` so the UI can show feedback.
- New logic goes into a pure-Python module first, the node only wires ROS to it.
- rclpy logging: one call site = one severity. Use `if ok: log.info(...) else: log.warn(...)`,
  never `(log.info if ok else log.warn)(...)` — rclpy raises and the node dies.
