#!/usr/bin/env python3
"""Field odometry timing for simulation (ROS-free; 2026-11-13).

On the real boat `/blueboat/odom` is `robot_interface`'s republication of the
autopilot's EKF output received over the Base Station WiFi link: 3-4 Hz, or
bursts of 25 Hz mixed with 4 Hz stretches once the sonar processor has asked
the autopilot for faster messages, each sample late by the link's transport
jitter. Gazebo's odometry is a clean 20 Hz with no delay. This module decides,
for each raw Gazebo sample, whether it is forwarded and how late, so that the
controller (and every other consumer of `/blueboat/odom`) sees the arrival
pattern of the field.

Where the numbers come from (receive times only; no pose was read):
  * `field_4hz`, `field_13hz`: `/blueboat/odom` arrival intervals in the 22 run
    bags of 2026-10-27 and 2026-10-28 (6369 and 13787 intervals).
    `~/ros2_ws/sim_runs/sim_acceptance/04_control/timing/odom_timing.py`.
  * `field_3hz`: the odometry updates seen by the controller log of the six
    runs of 2026-09-07 used for identification (7211 intervals).
  * delay: receive time of the autopilot's ATTITUDE messages against their
    `time_boot_ms`, above the fastest deliveries, pooled over the same bags
    (21604 messages). `.../timing/link_delay.py`. The constant part of the
    delay cannot be measured without a common clock; `base_delay_s` (0 by
    default) is the place to put one.

Tables are deciles (5 %, 15 % ... 95 %): the 1 % tails (link stalls of
seconds) are deliberately left out; a stall is a fault case, not the timing of
a normal run.
"""

import random

# --- inter-arrival deciles [s] ------------------------------------------------
_DT_4HZ = (0.210, 0.234, 0.239, 0.241, 0.246, 0.253, 0.259, 0.261, 0.266, 0.289)
_DT_BURST = (0.008, 0.018, 0.020, 0.024, 0.034, 0.038, 0.040, 0.041, 0.043, 0.054)
_DT_3HZ = (0.300, 0.300, 0.300, 0.300, 0.350, 0.350, 0.350, 0.350, 0.350, 0.351)
# --- transport delay above the fastest delivery, deciles [s] ------------------
_DELAY = (0.000, 0.001, 0.001, 0.002, 0.002, 0.004, 0.008, 0.026, 0.095, 0.161)

# name: (deciles of the slow state, deciles of the burst state or None,
#        P(burst next | burst now), P(burst next | slow now))
PROFILES = {
    'field_3hz':  (_DT_3HZ, None, 0.0, 0.0),        # 2026-09-07 (3.0 Hz)
    'field_4hz':  (_DT_4HZ, None, 0.0, 0.0),        # 2026-10-27/28 without the sonar processor (3.9 Hz)
    'field_13hz': (_DT_4HZ, _DT_BURST, 0.98, 0.03),  # with it: 25 Hz bursts and 4 Hz stretches (8.6 Hz mean)
}
# 'off' would be read as a YAML boolean on a command line (odom_profile:=off -> false)
OFF = 'passthrough'


def profile_names():
    return (OFF,) + tuple(PROFILES)


class OdomEmulator:
    """Decides which raw odometry samples are forwarded, and when.

    `offer(stamp)` is called for every raw sample with its (simulation) time
    stamp; it returns the time at which that sample is to be released, or None
    when the sample is dropped. Releases never go backwards in time (a late
    sample delays the ones behind it, as on a link). Deterministic for a seed.
    """

    def __init__(self, profile, seed=0, base_delay_s=0.0):
        if profile not in PROFILES:
            raise ValueError(f"unknown odometry profile '{profile}' ({' | '.join(profile_names())})")
        self.profile = profile
        self.slow, self.burst, self.p_bb, self.p_sb = PROFILES[profile]
        self.base_delay_s = float(base_delay_s)
        self.rng = random.Random(int(seed))
        self.in_burst = False
        self.next_due = None          # stamp from which the next sample is taken
        self.last_release = None

    def _draw(self, deciles):
        """Inverse-CDF draw from a decile table (linear inside a decile)."""
        u = self.rng.random() * (len(deciles) - 1)
        i = int(u)
        return deciles[i] + (u - i) * (deciles[i + 1] - deciles[i]) if i + 1 < len(deciles) else deciles[-1]

    def _interval(self):
        if self.burst is not None:
            p = self.p_bb if self.in_burst else self.p_sb
            self.in_burst = self.rng.random() < p
            if self.in_burst:
                return self._draw(self.burst)
        return self._draw(self.slow)

    def offer(self, stamp):
        stamp = float(stamp)
        if self.next_due is not None and stamp < self.next_due - 1e-9:
            if stamp < self.next_due - 5.0:        # clock went back (simulation reset)
                self.next_due = None
                self.last_release = None
            else:
                return None
        # The next sample is due one drawn interval after THIS one was due
        # (not after it arrived), so the mean rate is the profile's and not
        # one raw period slower; never scheduled in the past.
        due = stamp if self.next_due is None else self.next_due
        self.next_due = max(due + self._interval(), stamp)
        release = stamp + self.base_delay_s + self._draw(_DELAY)
        if self.last_release is not None and release < self.last_release:
            release = self.last_release
        self.last_release = release
        return release
