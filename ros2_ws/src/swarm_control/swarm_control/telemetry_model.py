"""Pure-Python telemetry model (no ROS): battery, link, flight mode, status, warnings, faults.

swarm_telemetry.py feeds it positions (from /tf) and the commander's view, calls
step() at ~10 Hz and copies the result into swarm_interfaces/DroneState.
Kept free of rclpy so it can be unit-tested.
"""
import math
import random
from dataclasses import dataclass, field

# --- must match swarm_interfaces/msg/DroneState.msg -------------------------
STATUS_IDLE, STATUS_FLYING, STATUS_SEARCHING = 'idle', 'flying', 'searching'
STATUS_RETURNING, STATUS_FAULT = 'returning', 'fault'
MISSION_STATUSES = (STATUS_SEARCHING, STATUS_RETURNING)

MODE_LANDED, MODE_TAKING_OFF, MODE_HOVERING = 'landed', 'taking_off', 'hovering'
MODE_MOVING, MODE_LANDING = 'moving', 'landing'

WARN_BATTERY_LOW = 1
WARN_BATTERY_CRITICAL = 2
WARN_LINK_WEAK = 4
WARN_SEPARATION = 8
WARN_GEOFENCE = 16
WARN_POSITION_STALE = 32

FAULT_BATTERY_DEPLETED = 1
FAULT_LINK_LOST = 2
FAULT_MOTOR = 4
FAULT_SENSOR = 8
FAULT_POSITION_LOST = 16

WARNING_NAMES = {
    WARN_BATTERY_LOW: 'battery low',
    WARN_BATTERY_CRITICAL: 'battery critical',
    WARN_LINK_WEAK: 'weak link',
    WARN_SEPARATION: 'too close to another drone',
    WARN_GEOFENCE: 'outside flight area',
    WARN_POSITION_STALE: 'position not updating',
}
FAULT_NAMES = {
    FAULT_BATTERY_DEPLETED: 'battery depleted',
    FAULT_LINK_LOST: 'link lost',
    FAULT_MOTOR: 'motor failure',
    FAULT_SENSOR: 'sensor failure',
    FAULT_POSITION_LOST: 'position lost',
}

# Injectable faults (InjectFault.srv) and their default `value`
INJECTABLE = {
    'battery': 100.0,        # set battery percent
    'battery_drain': 10.0,   # drain multiplier
    'link_weak': 0.3,        # held link quality
    'link_loss': 0.0,
    'motor': 0.0,
    'sensor': 0.0,
    'position_loss': 0.0,
}

# commander phase -> flight mode (flying is split into hovering / moving)
_PHASE_MODE = {'landed': MODE_LANDED, 'taking_off': MODE_TAKING_OFF,
               'landing': MODE_LANDING}

# 1S LiPo open-circuit voltage vs. state of charge (approximate)
_OCV = [(0, 3.00), (5, 3.30), (10, 3.50), (20, 3.65), (40, 3.75),
        (60, 3.85), (80, 4.00), (100, 4.20)]


def battery_voltage(percent, under_load=False, sag=0.15):
    """Approximate 1S LiPo voltage for a state of charge (0..100 %)."""
    p = min(max(percent, 0.0), 100.0)
    for (p0, v0), (p1, v1) in zip(_OCV, _OCV[1:]):
        if p <= p1:
            v = v0 + (v1 - v0) * (p - p0) / (p1 - p0)
            break
    return v - sag if under_load else v


def bits_to_text(bits, names):
    return [text for bit, text in names.items() if bits & bit]


@dataclass
class TelemetryConfig:
    # battery (Crazyflie 2.1: ~7 min hover on a 250 mAh 1S LiPo)
    initial_battery: float = 100.0
    hover_endurance: float = 420.0     # s from 100 % to 0 % while hovering
    move_drain_factor: float = 0.3     # extra drain per m/s of speed (fraction of hover drain)
    idle_drain: float = 0.005          # %/s on the ground (electronics only)
    drain_scale: float = 1.0           # global multiplier, >1 = faster tests
    recharge_on_ground: bool = False   # True = battery refills while landed (dev convenience)
    recharge_rate: float = 2.0         # %/s when recharge_on_ground
    battery_low: float = 30.0          # %
    battery_critical: float = 15.0     # %
    battery_depleted: float = 3.0      # %
    # radio link (simple distance model from a ground station)
    ground_station: tuple = (0.0, 0.0, 0.0)
    link_good_range: float = 4.0       # m, quality ~1 inside
    link_max_range: float = 12.0       # m, quality 0 at / beyond
    link_noise: float = 0.02
    link_weak: float = 0.5
    # timing
    stale_timeout: float = 0.5         # s without a new position -> warning
    lost_timeout: float = 2.0          # s without a new position -> fault
    commander_timeout: float = 1.0     # s, after that flight mode is inferred from motion
    # motion
    velocity_filter: float = 0.5       # low-pass weight of the newest sample (0..1]
    moving_speed: float = 0.05         # m/s, above = moving
    ground_height: float = 0.05        # m, below (and slow) = landed when inferring
    # safety (same values as swarm_commander)
    min_separation: float = 0.3
    fence_lo: tuple = (-1.0, -1.0, 0.2)
    fence_hi: tuple = (3.0, 2.0, 2.0)


@dataclass
class DroneTelemetry:
    id: str
    cfg: TelemetryConfig
    position: tuple = (0.0, 0.0, 0.0)
    velocity: tuple = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    battery: float = 100.0
    link_quality: float = 1.0
    flight_mode: str = MODE_LANDED
    status: str = STATUS_IDLE
    mission: str = ''                  # '' | searching | returning
    warnings: int = 0
    faults: int = 0
    has_target: bool = False
    target: tuple = (0.0, 0.0, 0.0)
    nearest_neighbour: float = 0.0
    data_time: float = None            # our clock when the newest position arrived
    injected: dict = field(default_factory=dict)
    _sample_stamp: float = None        # source stamp of the newest position
    _commander: tuple = None           # (phase, has_target, target, our time)
    _first_step: float = None          # our clock at the first step (startup grace)
    _rng: random.Random = None

    def __post_init__(self):
        self.battery = self.cfg.initial_battery
        self._rng = random.Random(sum(map(ord, self.id)))

    # ------------------------------------------------------------- inputs
    @property
    def link_up(self):
        return 'link_loss' not in self.injected

    def update_pose(self, position, yaw, stamp, now):
        """New position sample. `stamp` = source time (tf header), `now` = our clock.

        Velocity uses stamp differences, freshness uses our clock, so the sim and
        this node do not need to share a clock. Ignored while the link or the
        position source is (simulated) down.
        """
        if not self.link_up or 'position_loss' in self.injected:
            return
        if self._sample_stamp is not None and stamp == self._sample_stamp:
            return  # same sample as last time
        if self._sample_stamp is not None and self.data_time is not None:
            dt = stamp - self._sample_stamp
            if dt <= 0 or dt > 1.0:
                dt = now - self.data_time
            if dt > 1e-3:
                raw = tuple((a - b) / dt for a, b in zip(position, self.position))
                k = self.cfg.velocity_filter
                self.velocity = tuple(k * r + (1 - k) * v for r, v in zip(raw, self.velocity))
        self.position = tuple(position)
        self.yaw = yaw
        self._sample_stamp = stamp
        self.data_time = now

    def update_commander(self, phase, has_target, target, now):
        if not self.link_up:
            return
        self._commander = (phase, has_target, tuple(target), now)

    @property
    def speed(self):
        return math.sqrt(sum(v * v for v in self.velocity))

    @property
    def airborne(self):
        return self.flight_mode != MODE_LANDED

    def data_age(self, now):
        """Seconds since the newest position; before the first one, since startup."""
        if self.data_time is not None:
            return now - self.data_time
        return 0.0 if self._first_step is None else now - self._first_step

    # --------------------------------------------------------------- step
    def step(self, now, dt):
        """Advance battery + link by dt and recompute mode, flags and status."""
        c = self.cfg
        if self._first_step is None:
            self._first_step = now
        self.update_flight_mode(now)
        self._update_battery(dt)
        self._update_link(dt)

        w = 0
        if self.battery < c.battery_critical:
            w |= WARN_BATTERY_CRITICAL
        elif self.battery < c.battery_low:
            w |= WARN_BATTERY_LOW
        if self.link_up and self.link_quality < c.link_weak:
            w |= WARN_LINK_WEAK
        if self.airborne and 0.0 < self.nearest_neighbour < c.min_separation:
            w |= WARN_SEPARATION
        if self._outside_fence():
            w |= WARN_GEOFENCE
        age = self.data_age(now)
        if age > c.stale_timeout:
            w |= WARN_POSITION_STALE

        f = 0
        if self.battery <= c.battery_depleted:
            f |= FAULT_BATTERY_DEPLETED
        if not self.link_up or self.link_quality <= 0.0:
            f |= FAULT_LINK_LOST
        if 'motor' in self.injected:
            f |= FAULT_MOTOR
        if 'sensor' in self.injected:
            f |= FAULT_SENSOR
        if age > c.lost_timeout:
            f |= FAULT_POSITION_LOST
        self.warnings, self.faults = w, f

        if not self.airborne:
            self.mission = ''  # a landed drone has finished its task
        if f:
            self.status = STATUS_FAULT
        elif not self.airborne:
            self.status = STATUS_IDLE
        elif self.mission in MISSION_STATUSES:
            self.status = self.mission
        else:
            self.status = STATUS_FLYING

    def alerts(self):
        out = bits_to_text(self.faults, FAULT_NAMES)
        for t in bits_to_text(self.warnings, WARNING_NAMES):
            if t.startswith('battery'):
                t += f' ({self.battery:.0f}%)'
            elif t == 'weak link':
                t += f' ({self.link_quality:.2f})'
            elif t.startswith('too close'):
                t += f' ({self.nearest_neighbour:.2f} m)'
            out.append(t)
        return out

    def voltage(self):
        return battery_voltage(self.battery, under_load=self.airborne)

    # ---------------------------------------------------------- internals
    def update_flight_mode(self, now):
        c = self.cfg
        cmd = self._commander
        if cmd is not None and now - cmd[3] <= c.commander_timeout:
            phase, self.has_target, self.target = cmd[0], cmd[1], cmd[2]
            if phase in _PHASE_MODE:
                self.flight_mode = _PHASE_MODE[phase]
            elif self.has_target or self.speed > c.moving_speed:
                self.flight_mode = MODE_MOVING
            else:
                self.flight_mode = MODE_HOVERING
            return
        # no commander: infer from motion (e.g. drones driven by another node)
        self.has_target = False
        if self.position[2] < c.ground_height and self.speed < c.moving_speed:
            self.flight_mode = MODE_LANDED
        elif self.speed > c.moving_speed:
            self.flight_mode = MODE_MOVING
        else:
            self.flight_mode = MODE_HOVERING

    def _update_battery(self, dt):
        c = self.cfg
        if not self.airborne and c.recharge_on_ground:
            self.battery = min(100.0, self.battery + c.recharge_rate * dt)
            return
        if self.airborne:
            hover = 100.0 / c.hover_endurance
            rate = hover * (1.0 + c.move_drain_factor * self.speed)
        else:
            rate = c.idle_drain
        rate *= c.drain_scale * self.injected.get('battery_drain', 1.0)
        self.battery = max(0.0, self.battery - rate * dt)

    def _update_link(self, dt):
        c = self.cfg
        if not self.link_up:
            self.link_quality = 0.0
            return
        if 'link_weak' in self.injected:
            base = self.injected['link_weak']
        else:
            d = math.dist(self.position, c.ground_station)
            span = max(c.link_max_range - c.link_good_range, 1e-6)
            base = 1.0 - min(max((d - c.link_good_range) / span, 0.0), 1.0)
        noisy = base + self._rng.gauss(0.0, c.link_noise)
        k = min(1.0, dt / 0.3) if dt > 0 else 1.0  # ~0.3 s smoothing
        self.link_quality = min(1.0, max(0.0, (1 - k) * self.link_quality + k * noisy))

    def _outside_fence(self):
        c = self.cfg
        if not self.airborne:
            return False
        check_z = self.flight_mode in (MODE_HOVERING, MODE_MOVING)
        for i, (v, lo, hi) in enumerate(zip(self.position, c.fence_lo, c.fence_hi)):
            if i == 2 and not check_z:
                continue
            if v < lo - 0.05 or v > hi + 0.05:  # small tolerance for tracking error
                return True
        return False

    # ------------------------------------------------------- fault inject
    def inject(self, fault, active=True, value=0.0):
        """Apply / clear an injected fault. Returns a short description."""
        fault = fault.strip().lower()
        if fault == 'all':
            if active:
                raise ValueError('"all" only works with active=false (clear everything)')
            self.injected.clear()
            return 'cleared all'
        if fault not in INJECTABLE:
            raise ValueError(f'unknown fault "{fault}" (use {", ".join(INJECTABLE)}, all)')
        val = value if value > 0 else INJECTABLE[fault]
        if fault == 'battery':
            self.battery = min(max(val, 0.0), 100.0) if active else 100.0
            return f'battery {self.battery:.0f}%'
        if not active:
            self.injected.pop(fault, None)
            if fault == 'link_loss':
                self.link_quality = 0.0  # recovers smoothly
            return f'{fault} cleared'
        if fault == 'link_weak':
            val = min(max(val, 0.0), 1.0)
        self.injected[fault] = val
        return fault if fault in ('link_loss', 'motor', 'sensor', 'position_loss') \
            else f'{fault}={val:g}'

    def set_mission(self, status):
        status = status.strip().lower()
        if status in ('', 'none', STATUS_FLYING, STATUS_IDLE):
            self.mission = ''
            return 'none'
        if status not in MISSION_STATUSES:
            raise ValueError(f'unknown status "{status}" (use searching, returning, none)')
        if not self.airborne:
            raise ValueError(f'{self.id} is landed')
        self.mission = status
        return status


def nearest_neighbours(drones):
    """Set nearest_neighbour on every drone; return (min airborne separation, pair)."""
    ds = list(drones)
    for d in ds:
        others = [math.dist(d.position, o.position) for o in ds if o is not d
                  and o.airborne]
        d.nearest_neighbour = min(others) if others else 0.0
    best, pair = math.inf, None
    air = [d for d in ds if d.airborne]
    for i, a in enumerate(air):
        for b in air[i + 1:]:
            s = math.dist(a.position, b.position)
            if s < best:
                best, pair = s, (a.id, b.id)
    return best, pair
