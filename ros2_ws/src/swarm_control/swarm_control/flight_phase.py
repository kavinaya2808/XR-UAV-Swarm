"""Commander flight phases, driven by time AND the measured height (pure Python, no ROS).

A phase only advances when the drone has really done it: the sim (and real
Crazyflies) start a takeoff a little after the service call, so a timer alone
declares "flying" while the drone is still on the ground.
"""
LANDED, TAKING_OFF, FLYING, LANDING = 'landed', 'taking_off', 'flying', 'landing'


class PhaseConfig:
    def __init__(self, reached_fraction=0.8, ground_height=0.08, airborne_height=0.15,
                 timeout=3.0):
        self.reached_fraction = reached_fraction  # takeoff done at 80 % of the target height
        self.ground_height = ground_height        # m, below = on the ground
        self.airborne_height = airborne_height    # m, above = definitely flying
        self.timeout = timeout                    # s after the planned end -> give up waiting


def next_phase(phase, now, until, z, target_z, cfg=None):
    """Return (new_phase, note). note is None or a short text worth logging.

    phase     current phase
    now       current time (s)
    until     planned end of takeoff / landing (s)
    z         measured height (m), None if unknown (no /tf this tick)
    target_z  takeoff height (m)
    """
    c = cfg or PhaseConfig()
    if z is None:
        return phase, None
    late = now >= until + c.timeout
    if phase == TAKING_OFF and now >= until:
        if z >= c.reached_fraction * target_z:
            return FLYING, None
        if late:
            if z > c.airborne_height:
                return FLYING, f'takeoff slow (z {z:.2f} m), treating as flying'
            return LANDED, f'takeoff failed (still at z {z:.2f} m)'
    elif phase == LANDING and now >= until:
        if z <= c.ground_height:
            return LANDED, None
        if late:
            return LANDED, f'landing slow (z {z:.2f} m), treating as landed'
    elif phase == LANDED and z > c.airborne_height:
        # e.g. commander restarted while drones fly, or another node took off
        return FLYING, f'detected airborne at z {z:.2f} m'
    elif phase == FLYING and z < c.ground_height * 0.5:
        return LANDED, f'detected on the ground (z {z:.2f} m)'
    return phase, None
