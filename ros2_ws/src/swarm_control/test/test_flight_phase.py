"""Unit tests for flight_phase (no ROS needed): pytest test/test_flight_phase.py"""
from swarm_control.flight_phase import FLYING, LANDED, LANDING, TAKING_OFF, next_phase


def test_takeoff_waits_for_height():
    # planned end reached, but the drone is still low (sim starts late) -> keep taking_off
    assert next_phase(TAKING_OFF, 10.0, 10.0, 0.05, 0.5) == (TAKING_OFF, None)
    assert next_phase(TAKING_OFF, 10.5, 10.0, 0.45, 0.5) == (FLYING, None)


def test_takeoff_not_before_planned_end():
    assert next_phase(TAKING_OFF, 9.0, 10.0, 0.5, 0.5)[0] == TAKING_OFF


def test_takeoff_timeout():
    phase, note = next_phase(TAKING_OFF, 13.5, 10.0, 0.3, 0.5)
    assert phase == FLYING and 'slow' in note
    phase, note = next_phase(TAKING_OFF, 13.5, 10.0, 0.0, 0.5)
    assert phase == LANDED and 'failed' in note


def test_landing_waits_for_ground():
    assert next_phase(LANDING, 10.0, 10.0, 0.3, 0.5)[0] == LANDING
    assert next_phase(LANDING, 10.2, 10.0, 0.02, 0.5) == (LANDED, None)
    assert next_phase(LANDING, 13.5, 10.0, 0.3, 0.5)[0] == LANDED


def test_sync_with_reality():
    phase, note = next_phase(LANDED, 0.0, 0.0, 0.8, 0.5)   # commander restarted mid-flight
    assert phase == FLYING and note
    phase, note = next_phase(FLYING, 0.0, 0.0, 0.0, 0.5)
    assert phase == LANDED and note
    assert next_phase(FLYING, 0.0, 0.0, 1.0, 0.5) == (FLYING, None)
    assert next_phase(LANDED, 0.0, 0.0, 0.0, 0.5) == (LANDED, None)


def test_unknown_height_keeps_phase():
    assert next_phase(TAKING_OFF, 99.0, 10.0, None, 0.5) == (TAKING_OFF, None)
