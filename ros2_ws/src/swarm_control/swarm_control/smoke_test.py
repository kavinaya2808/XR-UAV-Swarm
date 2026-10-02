"""End-to-end smoke test against the running bringup (sim). Takes ~1 minute.

    ros2 launch swarm_control swarm_bringup.launch.py      # terminal 1
    ros2 run swarm_control smoke_test                       # terminal 2
    ros2 run swarm_control smoke_test --skip-faults         # flight checks only

Flies the drones (takeoff -> grid -> line -> land) and injects faults, so only
run it in simulation. Prints PASS / FAIL per check, exit code 0 = all passed.
Leaves the swarm landed with every injected fault cleared.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from swarm_interfaces.msg import DroneState, SwarmState
from swarm_interfaces.srv import (
    Formation, GoTo, InjectFault, SetMissionStatus, SwarmCommand,
)

GREEN, RED, DIM, RESET = '\033[32m', '\033[31m', '\033[2m', '\033[0m'
CENTER = Point(x=1.0, y=0.5, z=1.0)


class Abort(Exception):
    """A service disappeared mid-test (node crashed) -> stop, don't fly blind."""


class SmokeTest(Node):

    def __init__(self, args):
        super().__init__('swarm_smoke_test')
        self.args = args
        self.state = None
        self.stamps = []
        self.results = []
        self.create_subscription(SwarmState, '/swarm/state', self._on_state, 10)
        self.cli = {
            'takeoff': self.create_client(SwarmCommand, '/swarm/takeoff'),
            'land': self.create_client(SwarmCommand, '/swarm/land'),
            'formation': self.create_client(Formation, '/swarm/formation'),
            'go_to': self.create_client(GoTo, '/swarm/go_to'),
            'inject': self.create_client(InjectFault, '/swarm/inject_fault'),
            'mission': self.create_client(SetMissionStatus, '/swarm/set_mission_status'),
        }

    # ------------------------------------------------------------- plumbing
    def _on_state(self, msg):
        self.state = msg
        self.stamps.append(time.monotonic())

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, cond, timeout):
        """Spin until cond(state) is true. Returns True / False (timeout)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is not None and cond(self.state):
                return True
        return False

    def call(self, name, **fields):
        cli = self.cli[name]
        req = cli.srv_type.Request()
        for k, v in fields.items():
            setattr(req, k, v)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            if not cli.wait_for_service(timeout_sec=1.0):
                raise Abort(f'{cli.srv_name} disappeared during the test - the node behind '
                            f'it crashed. Check the bringup terminal for a traceback.')
            return False, 'no reply within 5 s'
        res = fut.result()
        return res.success, res.message

    def check(self, name, ok, detail=''):
        self.results.append((name, bool(ok)))
        mark = f'{GREEN}PASS{RESET}' if ok else f'{RED}FAIL{RESET}'
        print(f'  {mark}  {name}' + (f'  {DIM}{detail}{RESET}' if detail else ''))
        return ok

    def drones(self):
        return list(self.state.drones) if self.state else []

    def describe(self, only_problems=True):
        lines = []
        for d in self.drones():
            if only_problems and not (d.warnings or d.faults):
                continue
            p = d.position
            lines.append(f'{d.id}: pos ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) {d.flight_mode}/'
                         f'{d.status} bat {d.battery_percent:.0f}% link '
                         f'{d.link_quality:.2f} W={d.warnings} F={d.faults} {list(d.alerts)}')
        return '\n        '.join(lines)

    def all_(self, pred):
        return lambda s: len(s.drones) > 0 and all(pred(d) for d in s.drones)

    # ---------------------------------------------------------------- steps
    def run(self):
        print('\n== 1. System up')
        missing = [n for n, c in self.cli.items() if not c.wait_for_service(timeout_sec=10.0)]
        if not self.check('all /swarm services available', not missing,
                          f'missing: {missing}' if missing else ''):
            if len(missing) == len(self.cli):
                print('        -> nothing found: is the bringup running? Start it first '
                      '(swarm_up on the host, or ros2 launch swarm_control '
                      'swarm_bringup.launch.py) and check `ros2 node list`.')
            return
        if not self.check('/swarm/state received', self.wait_until(lambda s: True, 5.0)):
            return
        self.stamps.clear()
        self.spin_for(3.0)
        hz = (len(self.stamps) - 1) / (self.stamps[-1] - self.stamps[0]) \
            if len(self.stamps) > 2 else 0.0
        self.check('/swarm/state rate 8-12 Hz', 8.0 <= hz <= 12.0, f'{hz:.1f} Hz')
        n = len(self.drones())
        self.check('drone count matches num_drones', n > 0 and n == self.state.num_drones,
                   f'{n} drones: {", ".join(d.id for d in self.drones())}')

        # clean start: clear faults, full batteries, land anything still flying
        self.call('inject', fault='all', active=False)
        self.call('inject', fault='battery', active=False)
        if self.state.num_airborne:
            self.call('land')
            self.wait_until(self.all_(lambda d: d.flight_mode == DroneState.MODE_LANDED), 8.0)
        self.spin_for(1.0)
        self.check('on the ground: all idle, no warnings/faults',
                   self.all_(lambda d: d.status == DroneState.STATUS_IDLE and not d.warnings
                             and not d.faults)(self.state), self.describe())

        print('\n== 2. Take off')
        ok, msg = self.call('takeoff')
        self.check('/swarm/takeoff accepted', ok, msg)
        ok = self.wait_until(self.all_(lambda d: d.flight_mode == DroneState.MODE_HOVERING), 8.0)
        self.check('all hovering within 8 s', ok, self.describe(False) if not ok else '')
        self.check('height ~0.5 m',
                   all(abs(d.position.z - 0.5) < 0.1 for d in self.drones()),
                   ', '.join(f'{d.id} {d.position.z:.2f}' for d in self.drones()))
        self.check('status flying', self.all_(
            lambda d: d.status == DroneState.STATUS_FLYING)(self.state))
        self.spin_for(1.0)
        self.check('no warnings while hovering', self.state.num_warnings == 0
                   and self.state.num_faults == 0, self.describe())

        print('\n== 3. Formations')
        bat0 = {d.id: d.battery_percent for d in self.drones()}
        for shape in ('grid', 'line'):
            ok, msg = self.call('formation', shape=shape, center=CENTER)
            self.check(f'{shape}: accepted', ok, msg)
            seen = {'moving': False, 'peak': 0.0}

            def arrived(s):
                seen['moving'] |= any(d.flight_mode == DroneState.MODE_MOVING for d in s.drones)
                seen['peak'] = max([seen['peak']] + [d.speed for d in s.drones])
                return seen['moving'] and all(d.flight_mode == DroneState.MODE_HOVERING
                                              and not d.has_target for d in s.drones)
            ok = self.wait_until(arrived, 8.0)
            self.check(f'{shape}: drones report moving + velocity',
                       seen['moving'] and seen['peak'] > 0.05,
                       f'max speed {seen["peak"]:.2f} m/s')
            self.check(f'{shape}: arrived and hovering at z ~1.0', ok and all(
                abs(d.position.z - 1.0) < 0.1 for d in self.drones()))
            self.spin_for(0.5)
            self.check(f'{shape}: no warnings in formation', self.state.num_warnings == 0,
                       self.describe())
        drained = all(d.battery_percent < bat0[d.id] for d in self.drones())
        self.check('battery drains in flight', drained, ', '.join(
            f'{d.id} {bat0[d.id]:.1f}->{d.battery_percent:.1f}%' for d in self.drones()))

        if not self.args.skip_faults:
            self.fault_tests()

        print('\n== 5. Land')
        ok, msg = self.call('land')
        self.check('/swarm/land accepted', ok, msg)
        ok = self.wait_until(self.all_(lambda d: d.flight_mode == DroneState.MODE_LANDED
                                       and d.status == DroneState.STATUS_IDLE), 8.0)
        self.check('all landed and idle', ok, self.describe(False) if not ok else '')

        if not self.args.skip_faults:
            first = self.drones()[0].id
            self.call('inject', drone_ids=[first], fault='motor')
            self.wait_until(lambda s: s.drones[0].faults & DroneState.FAULT_MOTOR, 2.0)
            ok, msg = self.call('takeoff', drone_ids=[first])
            self.check(f'takeoff refused for {first} with motor fault',
                       not ok and 'fault' in msg, msg)
            self.call('inject', fault='all', active=False)

    def fault_tests(self):
        print('\n== 4. Warnings, faults, mission status')
        ids = [d.id for d in self.drones()]
        a, b = ids[0], ids[1 % len(ids)]

        def drone(i):
            return lambda s: next(d for d in s.drones if d.id == i)

        self.call('inject', drone_ids=[b], fault='battery', value=20.0)
        ok = self.wait_until(lambda s: drone(b)(s).warnings & DroneState.WARN_BATTERY_LOW, 2.0)
        self.check(f'{b} battery 20% -> battery low warning', ok,
                   str(list(drone(b)(self.state).alerts)))
        self.call('inject', drone_ids=[b], fault='battery', active=False)

        self.call('inject', drone_ids=[b], fault='link_weak', value=0.3)
        ok = self.wait_until(lambda s: drone(b)(s).warnings & DroneState.WARN_LINK_WEAK, 3.0)
        self.check(f'{b} link_weak -> weak link warning', ok,
                   f'link {drone(b)(self.state).link_quality:.2f}')
        self.call('inject', drone_ids=[b], fault='link_weak', active=False)

        self.call('inject', drone_ids=[a], fault='motor')
        ok = self.wait_until(lambda s: drone(a)(s).status == DroneState.STATUS_FAULT, 2.0)
        self.check(f'{a} motor fault -> status fault', ok)
        p = drone(a)(self.state).position
        ok, msg = self.call('go_to', drone_ids=[a], targets=[Point(x=p.x, y=p.y, z=p.z + 0.2)])
        self.check(f'go_to refused for faulted {a}', not ok and 'fault' in msg, msg)
        self.call('inject', drone_ids=[a], fault='motor', active=False)
        ok = self.wait_until(lambda s: drone(a)(s).faults == 0, 2.0)
        self.check(f'{a} fault cleared', ok)

        ok, msg = self.call('mission', status='searching')
        seen = self.wait_until(self.all_(
            lambda d: d.status == DroneState.STATUS_SEARCHING), 2.0)
        self.check('set_mission_status searching -> all searching', ok and seen, msg)
        self.call('mission', status='none')
        ok = self.wait_until(self.all_(lambda d: d.status == DroneState.STATUS_FLYING), 2.0)
        self.check('mission status none -> flying again', ok)

        self.spin_for(1.0)
        self.check('no warnings/faults left', self.state.num_warnings == 0
                   and self.state.num_faults == 0, self.describe())

    def summary(self):
        failed = [n for n, ok in self.results if not ok]
        total = len(self.results)
        print()
        if failed:
            print(f'{RED}{len(failed)}/{total} checks FAILED:{RESET} ' + '; '.join(failed))
        else:
            print(f'{GREEN}All {total} checks passed.{RESET}')
        return not failed


def main(argv=None):
    parser = argparse.ArgumentParser(description='Swarm end-to-end smoke test (sim only)')
    parser.add_argument('--skip-faults', action='store_true',
                        help='only flight checks, no fault injection')
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    node = SmokeTest(args)
    ok = False
    try:
        node.run()
        ok = node.summary()
    except Abort as e:
        node.check('services stay alive', False, str(e))
        node.summary()
    except KeyboardInterrupt:
        print('\ninterrupted — landing')
    finally:
        try:  # always leave the swarm landed and clean
            node.call('inject', fault='all', active=False)
            if node.state is not None and node.state.num_airborne:
                node.call('land')
        except Exception:  # noqa: BLE001  (best effort, e.g. commander already dead)
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
