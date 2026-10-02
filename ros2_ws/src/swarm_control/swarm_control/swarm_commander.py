"""Swarm commander: the single entry point between the Unity MR UI and Crazyswarm2.

Services (all return success + message, so the UI can show feedback):
    /swarm/takeoff    swarm_interfaces/srv/SwarmCommand
    /swarm/land       swarm_interfaces/srv/SwarmCommand
    /swarm/go_to      swarm_interfaces/srv/GoTo
    /swarm/formation  swarm_interfaces/srv/Formation
    /swarm/stop       std_srvs/srv/Trigger   (hover in place)
Topics:
    /swarm/commander_state  swarm_interfaces/msg/CommanderState  (out: phase + target per drone)
    /swarm/state            swarm_interfaces/msg/SwarmState      (in: from swarm_telemetry)

Every command passes through the same safety layer: flight volume (fence),
minimum separation between planned positions, and a speed limit.
Health gating (if swarm_telemetry runs): drones with a fault cannot take off or
move, drones with a critical battery cannot take off. Landing is always allowed.
"""
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import GoTo as CfGoTo
from crazyflie_interfaces.srv import Land as CfLand
from crazyflie_interfaces.srv import Takeoff as CfTakeoff
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from swarm_interfaces.msg import CommanderDrone, CommanderState, DroneState, SwarmState
from swarm_interfaces.srv import Formation, GoTo, SwarmCommand
from tf2_ros import Buffer, TransformException, TransformListener

from swarm_control.flight_phase import FLYING, LANDED, LANDING, TAKING_OFF, next_phase
from swarm_control.geometry import (
    SHAPES, Fence, assign_slots, check_separation, flight_duration, formation_targets,
)


def to_duration(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))


class Drone:
    def __init__(self, node, name, initial_position):
        self.id = name
        self.position = tuple(initial_position)
        self.status = LANDED
        self.status_until = 0.0
        self.takeoff_z = 0.0          # height of the last takeoff command
        self.target = None
        self.target_until = 0.0
        self.health = None          # (DroneState from /swarm/state, receive time)
        self.cli_takeoff = node.create_client(CfTakeoff, f'/{name}/takeoff')
        self.cli_land = node.create_client(CfLand, f'/{name}/land')
        self.cli_goto = node.create_client(CfGoTo, f'/{name}/go_to')

    @property
    def airborne(self):
        return self.status != LANDED

    def health_problem(self, now, for_takeoff=False, max_age=2.0):
        """Reason this drone must not take off / move, or None (no telemetry = no gating)."""
        if self.health is None or now - self.health[1] > max_age:
            return None
        s = self.health[0]
        if s.faults:
            return f'{self.id} has a fault ({", ".join(s.alerts) or "fault"})'
        if for_takeoff and s.warnings & DroneState.WARN_BATTERY_CRITICAL:
            return f'{self.id} battery critical ({s.battery_percent:.0f}%)'
        return None

    def planned_position(self):
        """Where this drone will be once its current command finishes."""
        return self.target if self.target is not None else self.position


class SwarmCommander(Node):

    def __init__(self):
        super().__init__('swarm_commander')

        p = self.declare_parameter
        p('crazyflies_yaml_file', '')
        p('world_frame', 'world')
        p('state_rate', 10.0)
        p('fence.x_min', -1.0)
        p('fence.x_max', 3.0)
        p('fence.y_min', -1.0)
        p('fence.y_max', 2.0)
        p('fence.z_min', 0.2)
        p('fence.z_max', 2.0)
        p('fence_mode', 'clamp')          # clamp | reject
        p('min_separation', 0.3)          # m, between planned positions
        p('max_speed', 0.5)               # m/s
        p('min_duration', 1.0)            # s
        p('sync_group', True)             # group moves arrive together
        p('takeoff_height', 0.5)
        p('takeoff_duration', 2.0)
        p('land_duration', 2.0)
        p('default_spacing', 0.6)
        p('health_gating', True)          # use /swarm/state faults / battery

        g = self.get_parameter
        self.world = g('world_frame').value
        self.fence = Fence(g('fence.x_min').value, g('fence.x_max').value,
                           g('fence.y_min').value, g('fence.y_max').value,
                           g('fence.z_min').value, g('fence.z_max').value)
        self.fence_mode = g('fence_mode').value
        self.min_sep = g('min_separation').value
        self.max_speed = g('max_speed').value
        self.min_duration = g('min_duration').value
        self.sync_group = g('sync_group').value
        self.takeoff_height = g('takeoff_height').value
        self.takeoff_duration = g('takeoff_duration').value
        self.land_duration = g('land_duration').value
        self.default_spacing = g('default_spacing').value
        self.health_gating = g('health_gating').value

        self.drones = {}
        for name, pos in self._load_drones(g('crazyflies_yaml_file').value):
            self.drones[name] = Drone(self, name, pos)
        if not self.drones:
            raise RuntimeError('No enabled drones found — check crazyflies_yaml_file')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state_pub = self.create_publisher(CommanderState, '/swarm/commander_state', 10)
        self.create_subscription(SwarmState, '/swarm/state', self._on_swarm_state, 10)
        self.create_service(SwarmCommand, '/swarm/takeoff', self._on_takeoff)
        self.create_service(SwarmCommand, '/swarm/land', self._on_land)
        self.create_service(GoTo, '/swarm/go_to', self._on_go_to)
        self.create_service(Formation, '/swarm/formation', self._on_formation)
        self.create_service(Trigger, '/swarm/stop', self._on_stop)
        self.create_timer(1.0 / g('state_rate').value, self._tick)

        self.get_logger().info(
            f'swarm_commander ready: {len(self.drones)} drones '
            f'({", ".join(self.drones)}), fence {self.fence.lo}..{self.fence.hi}, '
            f'min_sep {self.min_sep} m, max_speed {self.max_speed} m/s')

    # ------------------------------------------------------------------ setup
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

    # ---------------------------------------------------------- periodic loop
    def _tick(self):
        now = self._now()
        for d in self.drones.values():
            z = None
            try:
                t = self.tf_buffer.lookup_transform(self.world, d.id, Time())
                tr = t.transform.translation
                d.position = (tr.x, tr.y, tr.z)
                z = tr.z
            except TransformException:
                pass  # keep last known / initial position, don't change phase
            phase, note = next_phase(d.status, now, d.status_until, z, d.takeoff_z)
            if note:
                self.get_logger().warn(f'{d.id}: {note}')
            if phase != d.status:
                d.status = phase
                if phase == LANDED:
                    d.target = None
            if d.target is not None and now >= d.target_until and d.status == FLYING:
                d.target = None

        # separation monitoring + warnings live in swarm_telemetry (/swarm/state)
        msg = CommanderState()
        msg.stamp = self.get_clock().now().to_msg()
        for d in self.drones.values():
            c = CommanderDrone(id=d.id, phase=d.status, has_target=d.target is not None)
            if d.target is not None:
                c.target = Point(x=d.target[0], y=d.target[1], z=d.target[2])
            msg.drones.append(c)
        self.state_pub.publish(msg)

    def _on_swarm_state(self, msg):
        now = self._now()
        for s in msg.drones:
            d = self.drones.get(s.id)
            if d is not None:
                d.health = (s, now)

    # ---------------------------------------------------------------- helpers
    def _resolve(self, ids, default_filter=None):
        """Turn a list of ids (empty = all) into Drone objects, or raise ValueError."""
        if not ids:
            ds = [d for d in self.drones.values() if default_filter is None or default_filter(d)]
            if not ds:
                raise ValueError('no drones match')
            return ds
        unknown = [i for i in ids if i not in self.drones]
        if unknown:
            raise ValueError(f'unknown drone(s): {", ".join(unknown)}')
        if len(set(ids)) != len(ids):
            raise ValueError('duplicate drone ids')
        return [self.drones[i] for i in ids]

    def _healthy(self, d, for_takeoff=False):
        if not self.health_gating:
            return None
        return d.health_problem(self._now(), for_takeoff)

    def _can_move(self, d):
        """Default selection for go_to / formation: flying and healthy."""
        return d.status == FLYING and not self._healthy(d)

    def _servers_ready(self, drones, attr):
        missing = [d.id for d in drones if not getattr(d, attr).service_is_ready()]
        if missing:
            raise ValueError(f'crazyflie_server not ready for {", ".join(missing)}')

    def _execute_go_to(self, drones, targets, requested_duration):
        """Safety-check and send absolute targets. Returns a status message."""
        notes = []
        for d in drones:
            if d.status != FLYING:
                raise ValueError(f'{d.id} is {d.status}; only flying drones can move')
            problem = self._healthy(d)
            if problem:
                raise ValueError(f'{problem}; land it instead')

        # 1) fence
        checked = []
        for d, t in zip(drones, targets):
            if not self.fence.contains(t):
                if self.fence_mode == 'reject':
                    raise ValueError(f'target for {d.id} {fmt(t)} is outside the flight area')
                t = self.fence.clamp(t)
                notes.append(f'{d.id} clamped to {fmt(t)}')
            checked.append(t)

        # 2) separation between all planned end positions
        planned = {o.id: o.planned_position() for o in self.drones.values() if o.airborne}
        for d, t in zip(drones, checked):
            planned[d.id] = t
        ok, pair, sep = check_separation(planned, self.min_sep)
        if not ok:
            raise ValueError(f'{pair[0]} and {pair[1]} would be {sep:.2f} m apart '
                             f'(min {self.min_sep} m)')

        # 3) speed limit -> durations
        durations = [flight_duration(d.position, t, self.max_speed, self.min_duration,
                                     requested_duration) for d, t in zip(drones, checked)]
        if self.sync_group:
            durations = [max(durations)] * len(durations)

        # 4) send
        now = self._now()
        for d, t, dur in zip(drones, checked, durations):
            req = CfGoTo.Request()
            req.group_mask = 0
            req.relative = False
            req.goal = Point(x=t[0], y=t[1], z=t[2])
            req.yaw = 0.0
            req.duration = to_duration(dur)
            d.cli_goto.call_async(req)
            d.target, d.target_until = t, now + dur
        notes.insert(0, f'moving {len(drones)} drone(s), {max(durations):.1f} s')
        return '; '.join(notes)

    # --------------------------------------------------------------- services
    def _on_takeoff(self, req, res):
        try:
            drones = self._resolve(list(req.drone_ids))
            self._servers_ready(drones, 'cli_takeoff')
            height = req.height if req.height > 0 else self.takeoff_height
            height = min(max(height, self.fence.lo[2]), self.fence.hi[2])
            duration = req.duration if req.duration > 0 else self.takeoff_duration
            now, sent, skipped, refused = self._now(), [], [], []
            for d in drones:
                if d.status != LANDED:
                    skipped.append(d.id)
                    continue
                problem = self._healthy(d, for_takeoff=True)
                if problem:
                    refused.append(problem)
                    continue
                r = CfTakeoff.Request(group_mask=0, height=float(height),
                                      duration=to_duration(duration))
                d.cli_takeoff.call_async(r)
                d.status, d.status_until = TAKING_OFF, now + duration
                d.takeoff_z = float(height)
                d.target = (d.position[0], d.position[1], height)
                d.target_until = now + duration
                sent.append(d.id)
            res.success = bool(sent)
            res.message = f'takeoff to {height:.2f} m: {", ".join(sent) or "none"}'
            if skipped:
                res.message += f' (already airborne: {", ".join(skipped)})'
            if refused:
                res.message += f' (refused: {"; ".join(refused)})'
        except ValueError as e:
            res.success, res.message = False, str(e)
        self._log(res)
        return res

    def _on_land(self, req, res):
        try:
            drones = self._resolve(list(req.drone_ids))
            self._servers_ready(drones, 'cli_land')
            duration = req.duration if req.duration > 0 else self.land_duration
            now, sent = self._now(), []
            for d in drones:
                if d.status == LANDED:
                    continue
                r = CfLand.Request(group_mask=0, height=0.0, duration=to_duration(duration))
                d.cli_land.call_async(r)
                d.status, d.status_until = LANDING, now + duration
                d.target = None
                sent.append(d.id)
            res.success = True
            res.message = f'landing: {", ".join(sent) or "none (all landed)"}'
        except ValueError as e:
            res.success, res.message = False, str(e)
        self._log(res)
        return res

    def _on_go_to(self, req, res):
        try:
            drones = self._resolve(list(req.drone_ids), default_filter=self._can_move)
            self._servers_ready(drones, 'cli_goto')
            pts = [(p.x, p.y, p.z) for p in req.targets]
            if req.relative:
                if len(pts) == 1:
                    pts = pts * len(drones)
                if len(pts) != len(drones):
                    raise ValueError('relative: give 1 offset or one per drone')
                targets = [tuple(a + b for a, b in zip(d.planned_position(), off))
                           for d, off in zip(drones, pts)]
            else:
                if len(pts) != len(drones):
                    raise ValueError(f'{len(drones)} drone(s) but {len(pts)} target(s)')
                targets = pts
            res.message = self._execute_go_to(drones, targets, req.duration)
            res.success = True
        except ValueError as e:
            res.success, res.message = False, str(e)
        self._log(res)
        return res

    def _on_formation(self, req, res):
        try:
            shape = req.shape.strip().lower()
            if shape not in SHAPES:
                raise ValueError(f'unknown shape "{req.shape}" (use {", ".join(SHAPES)})')
            drones = self._resolve(list(req.drone_ids), default_filter=self._can_move)
            self._servers_ready(drones, 'cli_goto')
            spacing = req.spacing if req.spacing > 0 else self.default_spacing
            spacing = max(spacing, self.min_sep)
            z = req.center.z if req.center.z > 0 else \
                sum(d.position[2] for d in drones) / len(drones)
            slots = formation_targets(shape, len(drones),
                                      (req.center.x, req.center.y, z), spacing, req.heading)
            perm = assign_slots([d.position for d in drones], slots)
            targets = [slots[perm[i]] for i in range(len(drones))]
            res.message = f'{shape} formation: ' + \
                self._execute_go_to(drones, targets, req.duration)
            res.success = True
        except ValueError as e:
            res.success, res.message = False, str(e)
        self._log(res)
        return res

    def _on_stop(self, req, res):
        """Hover in place: every flying drone holds its current position."""
        flying = [d for d in self.drones.values() if d.status == FLYING]
        now = self._now()
        for d in flying:
            r = CfGoTo.Request(group_mask=0, relative=False,
                               goal=Point(x=d.position[0], y=d.position[1], z=d.position[2]),
                               yaw=0.0, duration=to_duration(self.min_duration))
            d.cli_goto.call_async(r)
            d.target, d.target_until = d.position, now + self.min_duration
        res.success = True
        res.message = f'hovering: {", ".join(d.id for d in flying) or "none flying"}'
        self._log(res)
        return res

    def _log(self, res):
        # rclpy: one call site must always use the same severity -> two separate calls
        if res.success:
            self.get_logger().info(res.message)
        else:
            self.get_logger().warn(res.message)


def fmt(p):
    return '(' + ', '.join(f'{v:.2f}' for v in p) + ')'


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCommander()
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
