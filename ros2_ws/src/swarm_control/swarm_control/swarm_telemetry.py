"""Swarm telemetry: one structured state record per drone, for the whole swarm.

Topic (out):
    /swarm/state               swarm_interfaces/msg/SwarmState   (~10 Hz)
Topic (in):
    /tf                        positions world -> cfN (Crazyswarm2)
    /swarm/commander_state     swarm_interfaces/msg/CommanderState (flight phase, targets)
Services:
    /swarm/inject_fault        swarm_interfaces/srv/InjectFault       (testing)
    /swarm/set_mission_status  swarm_interfaces/srv/SetMissionStatus  (search / return)

Battery and radio link are simulated (the Crazyswarm2 sim backend has neither);
all logic lives in telemetry_model.py so it can be unit-tested without ROS.
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.time import Time
from swarm_interfaces.msg import CommanderState, DroneState, SwarmState
from swarm_interfaces.srv import InjectFault, SetMissionStatus
from tf2_ros import Buffer, TransformException, TransformListener

from swarm_control.telemetry_model import (
    FAULT_NAMES, WARN_SEPARATION, WARNING_NAMES, DroneTelemetry, TelemetryConfig, bits_to_text,
    nearest_neighbours,
)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SwarmTelemetry(Node):

    def __init__(self):
        super().__init__('swarm_telemetry')
        d = TelemetryConfig()
        p = self.declare_parameter
        p('crazyflies_yaml_file', '')
        p('world_frame', 'world')
        p('rate', 10.0)
        p('initial_battery', d.initial_battery)
        p('hover_endurance', d.hover_endurance)
        p('move_drain_factor', d.move_drain_factor)
        p('idle_drain', d.idle_drain)
        p('drain_scale', d.drain_scale)
        p('recharge_on_ground', d.recharge_on_ground)
        p('recharge_rate', d.recharge_rate)
        p('battery_low', d.battery_low)
        p('battery_critical', d.battery_critical)
        p('battery_depleted', d.battery_depleted)
        p('ground_station', list(d.ground_station))
        p('link_good_range', d.link_good_range)
        p('link_max_range', d.link_max_range)
        p('link_noise', d.link_noise)
        p('link_weak', d.link_weak)
        p('stale_timeout', d.stale_timeout)
        p('lost_timeout', d.lost_timeout)
        p('commander_timeout', d.commander_timeout)
        p('velocity_filter', d.velocity_filter)
        p('moving_speed', d.moving_speed)
        p('ground_height', d.ground_height)
        # shared with swarm_commander (config/swarm_limits.yaml)
        p('min_separation', d.min_separation)
        p('fence.x_min', d.fence_lo[0])
        p('fence.x_max', d.fence_hi[0])
        p('fence.y_min', d.fence_lo[1])
        p('fence.y_max', d.fence_hi[1])
        p('fence.z_min', d.fence_lo[2])
        p('fence.z_max', d.fence_hi[2])

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        fields = [f for f in TelemetryConfig.__dataclass_fields__
                  if f not in ('ground_station', 'fence_lo', 'fence_hi')]
        self.cfg = TelemetryConfig(**{f: g(f) for f in fields})
        self.cfg.ground_station = tuple(float(v) for v in g('ground_station'))
        self.cfg.fence_lo = (g('fence.x_min'), g('fence.y_min'), g('fence.z_min'))
        self.cfg.fence_hi = (g('fence.x_max'), g('fence.y_max'), g('fence.z_max'))
        self.world = g('world_frame')

        self.drones = {}
        for name, pos in self._load_drones(g('crazyflies_yaml_file')):
            t = DroneTelemetry(name, self.cfg)
            t.position = tuple(float(v) for v in pos)
            self.drones[name] = t
        if not self.drones:
            raise RuntimeError('No enabled drones found — check crazyflies_yaml_file')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(SwarmState, '/swarm/state', 10)
        self.create_subscription(CommanderState, '/swarm/commander_state',
                                 self._on_commander, 10)
        self.create_service(InjectFault, '/swarm/inject_fault', self._on_inject)
        self.create_service(SetMissionStatus, '/swarm/set_mission_status', self._on_mission)

        self._last_tick = None
        self._last_flags = {n: (0, 0) for n in self.drones}
        self._last_sep_warn = 0.0
        self.create_timer(1.0 / g('rate'), self._tick)

        self.get_logger().info(
            f'swarm_telemetry ready: {len(self.drones)} drones, {g("rate"):.0f} Hz, '
            f'hover endurance {self.cfg.hover_endurance:.0f} s x drain_scale '
            f'{self.cfg.drain_scale:g}, fence {self.cfg.fence_lo}..{self.cfg.fence_hi}')

    def _load_drones(self, path):
        if not path:
            self.get_logger().error('Parameter crazyflies_yaml_file is empty')
            return []
        with open(path) as f:
            cfg = yaml.safe_load(f)
        return [(name, r.get('initial_position', [0.0, 0.0, 0.0]))
                for name, r in cfg.get('robots', {}).items() if r.get('enabled', True)]

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ input
    def _on_commander(self, msg):
        now = self._now()
        for c in msg.drones:
            d = self.drones.get(c.id)
            if d is not None:
                d.update_commander(c.phase, c.has_target,
                                   (c.target.x, c.target.y, c.target.z), now)

    # ------------------------------------------------------------------- loop
    def _tick(self):
        now = self._now()
        dt = 0.0 if self._last_tick is None else min(now - self._last_tick, 1.0)
        self._last_tick = now

        for d in self.drones.values():
            try:
                t = self.tf_buffer.lookup_transform(self.world, d.id, Time())
            except TransformException:
                continue
            tr, q = t.transform.translation, t.transform.rotation
            d.update_pose((tr.x, tr.y, tr.z), yaw_from_quaternion(q),
                          stamp_to_sec(t.header.stamp), now)

        # flight mode first (separation only counts airborne drones), then the rest
        for d in self.drones.values():
            d.update_flight_mode(now)
        min_sep, pair = nearest_neighbours(self.drones.values())
        for d in self.drones.values():
            d.step(now, dt)

        self._log_changes(now, min_sep, pair)
        self.pub.publish(self._build_msg(min_sep))

    def _build_msg(self, min_sep):
        msg = SwarmState()
        msg.stamp = self.get_clock().now().to_msg()
        for d in self.drones.values():
            s = DroneState()
            s.stamp = Time(seconds=d.data_time).to_msg() if d.data_time else msg.stamp
            s.id = d.id
            s.position = Point(x=d.position[0], y=d.position[1], z=d.position[2])
            s.velocity = Vector3(x=d.velocity[0], y=d.velocity[1], z=d.velocity[2])
            s.speed = float(d.speed)
            s.yaw = float(d.yaw)
            s.battery_percent = float(d.battery)
            s.battery_voltage = float(d.voltage())
            s.flight_mode = d.flight_mode
            s.status = d.status
            s.warnings = d.warnings
            s.faults = d.faults
            s.alerts = d.alerts()
            s.link_quality = float(d.link_quality)
            s.has_target = d.has_target
            s.target = Point(x=d.target[0], y=d.target[1], z=d.target[2])
            s.nearest_neighbour = float(d.nearest_neighbour)
            msg.drones.append(s)
        ds = self.drones.values()
        msg.num_drones = len(self.drones)
        msg.num_airborne = sum(1 for d in ds if d.airborne)
        msg.num_warnings = sum(1 for d in ds if d.warnings)
        msg.num_faults = sum(1 for d in ds if d.faults)
        msg.min_separation = float(min_sep) if math.isfinite(min_sep) else 0.0
        msg.separation_warning = math.isfinite(min_sep) and min_sep < self.cfg.min_separation
        return msg

    def _log_changes(self, now, min_sep, pair):
        """Log each warning / fault once when it appears and when it clears."""
        log = self.get_logger()
        for d in self.drones.values():
            old_w, old_f = self._last_flags[d.id]
            for bit in bits_to_text(d.faults & ~old_f, FAULT_NAMES):
                log.error(f'{d.id}: FAULT {bit}')
            for bit in bits_to_text(old_f & ~d.faults, FAULT_NAMES):
                log.info(f'{d.id}: fault cleared — {bit}')
            for bit in bits_to_text(old_w & ~d.warnings & ~WARN_SEPARATION, WARNING_NAMES):
                log.info(f'{d.id}: warning cleared — {bit}')
            for bit in bits_to_text(d.warnings & ~old_w & ~WARN_SEPARATION, WARNING_NAMES):
                x, y, z = d.position
                log.warn(f'{d.id}: {bit} (pos {x:.2f},{y:.2f},{z:.2f}, {d.flight_mode}, '
                         f'battery {d.battery:.0f}%, link {d.link_quality:.2f})')
            self._last_flags[d.id] = (d.warnings, d.faults)
        if pair and min_sep < self.cfg.min_separation and now - self._last_sep_warn > 2.0:
            log.warn(f'separation {min_sep:.2f} m between {pair[0]} and {pair[1]} '
                     f'< {self.cfg.min_separation} m')
            self._last_sep_warn = now

    # --------------------------------------------------------------- services
    def _resolve(self, ids):
        if not ids:
            return list(self.drones.values())
        unknown = [i for i in ids if i not in self.drones]
        if unknown:
            raise ValueError(f'unknown drone(s): {", ".join(unknown)}')
        return [self.drones[i] for i in ids]

    def _on_inject(self, req, res):
        try:
            drones = self._resolve(list(req.drone_ids))
            results = [f'{d.id}: {d.inject(req.fault, req.active, req.value)}' for d in drones]
            res.success, res.message = True, '; '.join(results)
        except ValueError as e:
            res.success, res.message = False, str(e)
        text = f'inject_fault {req.fault} active={req.active}: {res.message}'
        # rclpy: one call site must always use the same severity -> two separate calls
        if res.success:
            self.get_logger().warn(text)
        else:
            self.get_logger().error(text)
        return res

    def _on_mission(self, req, res):
        try:
            drones = self._resolve(list(req.drone_ids))
            if not req.drone_ids:  # "all" = all airborne drones
                drones = [d for d in drones if d.airborne] or drones
            done = [f'{d.id}: {d.set_mission(req.status)}' for d in drones]
            res.success, res.message = True, '; '.join(done)
        except ValueError as e:
            res.success, res.message = False, str(e)
        self.get_logger().info(f'set_mission_status: {res.message}')
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SwarmTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
