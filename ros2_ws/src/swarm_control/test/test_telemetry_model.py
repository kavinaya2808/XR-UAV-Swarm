"""Unit tests for telemetry_model (no ROS needed): pytest test/test_telemetry_model.py"""
import pytest

from swarm_control import telemetry_model as tm


def drone(name='cf1', **cfg):
    d = tm.DroneTelemetry(name, tm.TelemetryConfig(link_noise=0.0, **cfg))
    d.update_pose((0.0, 0.0, 0.0), 0.0, stamp=0.0, now=0.0)
    return d


def fly(d, now, z=0.5, phase='flying', target=False):
    d.update_pose((0.0, 0.0, z), 0.0, stamp=now, now=now)
    d.update_commander(phase, target, (0.0, 0.0, z), now)
    d.update_flight_mode(now)
    d.step(now, 0.1)


def test_idle_on_ground():
    d = drone()
    d.update_commander('landed', False, (0, 0, 0), 0.0)
    d.step(0.1, 0.1)
    assert d.status == tm.STATUS_IDLE and d.flight_mode == tm.MODE_LANDED
    assert d.warnings == 0 and d.faults == 0
    assert d.battery > 99.9


def test_battery_drain_hover_matches_endurance():
    d = drone(hover_endurance=100.0)  # 1 %/s
    for i in range(1, 101):           # 10 s
        fly(d, i * 0.1)
    assert d.flight_mode == tm.MODE_HOVERING and d.status == tm.STATUS_FLYING
    assert d.battery == pytest.approx(90.0, abs=0.2)
    assert 3.7 < d.voltage() < 4.1


def test_battery_thresholds():
    d = drone()
    fly(d, 0.1)
    d.inject('battery', value=25)
    fly(d, 0.2)
    assert d.warnings & tm.WARN_BATTERY_LOW and not d.faults
    d.inject('battery', value=10)
    fly(d, 0.3)
    assert d.warnings & tm.WARN_BATTERY_CRITICAL
    assert not d.warnings & tm.WARN_BATTERY_LOW
    d.inject('battery', value=2)
    fly(d, 0.4)
    assert d.faults & tm.FAULT_BATTERY_DEPLETED and d.status == tm.STATUS_FAULT
    d.inject('battery', active=False)
    fly(d, 0.5)
    assert d.battery > 99.9 and d.status == tm.STATUS_FLYING


def test_battery_drain_multiplier():
    a, b = drone('cf1'), drone('cf2')
    b.inject('battery_drain', value=10)
    for i in range(1, 11):
        fly(a, i * 0.1)
        fly(b, i * 0.1)
    assert (100 - b.battery) == pytest.approx(10 * (100 - a.battery), rel=1e-6)


def test_velocity_and_moving_mode():
    d = drone()
    for i in range(1, 21):  # 0.5 m/s along x
        t = i * 0.1
        d.update_pose((0.05 * i, 0.0, 0.5), 0.0, stamp=t, now=t)
        d.update_commander('flying', True, (2.0, 0.0, 0.5), t)
        d.step(t, 0.1)
    assert d.velocity[0] == pytest.approx(0.5, abs=0.01)
    assert d.flight_mode == tm.MODE_MOVING


def test_link_weak_and_loss():
    d = drone()
    d.inject('link_weak', value=0.3)
    for i in range(1, 20):
        fly(d, i * 0.1)
    assert d.warnings & tm.WARN_LINK_WEAK and not d.faults
    d.inject('link_loss')
    last = d.position
    for i in range(20, 50):  # 3 s without data
        t = i * 0.1
        d.update_pose((1.0, 1.0, 1.0), 0.0, stamp=t, now=t)  # ignored while link down
        d.step(t, 0.1)
    assert d.position == last
    assert d.faults & tm.FAULT_LINK_LOST and d.faults & tm.FAULT_POSITION_LOST
    assert d.warnings & tm.WARN_POSITION_STALE
    assert d.status == tm.STATUS_FAULT
    d.inject('link_loss', active=False)
    d.inject('link_weak', active=False)
    for i in range(50, 70):
        fly(d, i * 0.1)
    assert d.faults == 0 and d.warnings == 0 and d.link_quality > 0.9


def test_link_distance_model():
    d = drone(link_good_range=4.0, link_max_range=12.0)
    d.update_pose((8.0, 0.0, 1.0), 0.0, stamp=0.1, now=0.1)
    d.update_commander('flying', False, (8, 0, 1), 0.1)
    d.step(0.1, 1.0)  # dt >= smoothing -> jumps to model value
    assert d.link_quality == pytest.approx(1 - (8.06 - 4) / 8, abs=0.01)


def test_motor_sensor_faults_and_clear_all():
    d = drone()
    fly(d, 0.1)
    d.inject('motor')
    d.inject('sensor')
    fly(d, 0.2)
    assert d.faults == tm.FAULT_MOTOR | tm.FAULT_SENSOR
    assert d.alerts()[:2] == ['motor failure', 'sensor failure']
    d.inject('all', active=False)
    fly(d, 0.3)
    assert d.faults == 0
    with pytest.raises(ValueError):
        d.inject('all', active=True)
    with pytest.raises(ValueError):
        d.inject('banana')


def test_position_stale_then_lost():
    d = drone()
    fly(d, 0.1)
    d.inject('position_loss')
    d.step(0.8, 0.1)
    assert d.warnings & tm.WARN_POSITION_STALE and not d.faults
    d.step(2.5, 0.1)
    assert d.faults & tm.FAULT_POSITION_LOST


def test_mission_status():
    d = drone()
    with pytest.raises(ValueError):
        d.set_mission('searching')  # landed
    fly(d, 0.1)
    d.set_mission('searching')
    fly(d, 0.2)
    assert d.status == tm.STATUS_SEARCHING
    d.set_mission('returning')
    fly(d, 0.3)
    assert d.status == tm.STATUS_RETURNING
    d.inject('motor')
    fly(d, 0.4)
    assert d.status == tm.STATUS_FAULT  # fault wins
    d.inject('motor', active=False)
    fly(d, 0.5, z=0.0, phase='landed')
    assert d.status == tm.STATUS_IDLE and d.mission == ''  # cleared on landing


def test_separation_and_geofence():
    a, b, c = drone('cf1'), drone('cf2'), drone('cf3')
    for d, x in ((a, 0.0), (b, 0.2), (c, 5.0)):
        d.update_pose((x, 0.0, 0.5), 0.0, stamp=0.1, now=0.1)
        d.update_commander('flying', False, (x, 0.0, 0.5), 0.1)
        d.update_flight_mode(0.1)
    best, pair = tm.nearest_neighbours([a, b, c])
    for d in (a, b, c):
        d.step(0.1, 0.1)
    assert pair == ('cf1', 'cf2') and best == pytest.approx(0.2)
    assert a.warnings & tm.WARN_SEPARATION and b.warnings & tm.WARN_SEPARATION
    assert not c.warnings & tm.WARN_SEPARATION
    assert c.warnings & tm.WARN_GEOFENCE  # x = 5 > 3


def test_landed_drone_not_geofenced_or_separated():
    a, b = drone('cf1'), drone('cf2')
    b.update_pose((0.1, 0.0, 0.0), 0.0, stamp=0.1, now=0.1)
    for d in (a, b):
        d.update_commander('landed', False, (0, 0, 0), 0.1)
        d.update_flight_mode(0.1)
    tm.nearest_neighbours([a, b])
    for d in (a, b):
        d.step(0.1, 0.1)
    assert a.warnings == 0 and b.warnings == 0  # z = 0 < fence z_min but landed


def test_flight_mode_inferred_without_commander():
    d = drone(commander_timeout=1.0)
    d.update_commander('flying', False, (0, 0, 0.5), 0.0)
    d.update_pose((0.0, 0.0, 0.5), 0.0, stamp=2.0, now=2.0)
    d.velocity = (0.0, 0.0, 0.0)
    d.step(2.0, 0.1)  # commander info is 2 s old -> inferred
    assert d.flight_mode == tm.MODE_HOVERING
    d.update_pose((0.0, 0.0, 0.0), 0.0, stamp=3.0, now=3.0)
    for i in range(10):
        d.step(3.0, 0.1)
    d.velocity = (0.0, 0.0, 0.0)
    d.step(3.0, 0.1)
    assert d.flight_mode == tm.MODE_LANDED


def test_voltage_curve():
    assert tm.battery_voltage(100) == pytest.approx(4.2)
    assert tm.battery_voltage(0) == pytest.approx(3.0)
    assert tm.battery_voltage(50, under_load=True) < tm.battery_voltage(50)


def test_no_position_fault_before_first_sample():
    d = tm.DroneTelemetry('cf1', tm.TelemetryConfig())
    d.step(100.0, 0.0)          # node just started, no /tf yet
    assert d.faults == 0 and d.warnings == 0
    d.step(101.0, 0.1)          # 1 s without data -> stale warning only
    assert d.warnings & tm.WARN_POSITION_STALE and not d.faults
    d.step(103.0, 0.1)          # 3 s without data -> position lost
    assert d.faults & tm.FAULT_POSITION_LOST
    d.update_pose((0.0, 0.0, 0.0), 0.0, stamp=1.0, now=103.1)
    d.step(103.1, 0.1)
    assert d.faults == 0 and d.warnings == 0
