#!/usr/bin/env python3

r"""
ArduPilot parameter mode switch: 'override' <-> 'default'.

Applies the SERVOn_FUNCTION mapping and the GCS system id that decide WHO is
allowed to drive the thrusters, then reports the result on
/blueboat/param_mode. robot_interface gates its whole control loop on that
echo, and the Mission Control Station uses it as the acknowledgement that a
safe-shutdown command landed (superproject CM-15).

WHY THIS NODE IS FULL OF WATCHDOGS
----------------------------------
Every MAVROS call here is a round trip to the autopilot over a wireless link.
The original version set `self.busy = True` before the first call and cleared
it ONLY inside a `call_async` done-callback, with no timeout on any of the
four calls. A `/mavros/param/pull` that never returned -- link drop, mavros
restart, a lost ACK -- latched `busy` forever, so every later request was
discarded with "Parameter sequence in progress, ignoring request" while
robot_interface re-requested every second, forever. Observed in the field as
an override that never locks, with no give-up and nothing on screen saying
why.

Three rules follow from that, and none of them may be removed:

  1. NO STATE IS EVER CLEARED ONLY BY A CALLBACK. `busy` is also cleared by a
     wall-clock watchdog (`param_sequence_timeout_s`). A sequence that stops
     making progress is abandoned, not waited on.
  2. EVERY ABANDONED SEQUENCE IS FENCED OFF BY A GENERATION COUNTER. A future
     that completes after its sequence was abandoned must not write state
     belonging to the sequence that replaced it - `_seq` is checked in every
     done-callback.
  3. THE NODE ALWAYS REPORTS. `publish_state()` publishes `param_mode` even
     when no mode has ever been applied (as the empty string), and a
     heartbeat repeats it, so "alive but not locked" is distinguishable from
     "dead" by a consumer that cannot see this node's log.
"""

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from mavros_msgs.srv import ParamPull
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

# --- ArduPilot SERVOn_FUNCTION values ---
SERVO_DISABLED = 0
RCIN1_PASSTHROUGH = 51    # servo outputs whatever RC channel 1 receives
RCIN3_PASSTHROUGH = 53    # servo outputs whatever RC channel 3 receives
THROTTLE_RIGHT = 74       # BlueBoat default mapping
THROTTLE_LEFT = 73

# --- GCS system-id handling ---
# ArduPilot only accepts RC_CHANNELS_OVERRIDE (and MANUAL_CONTROL) from the GCS
# whose MAVLink system id matches SYSID_MYGCS. QGC uses 255 (the default), which
# is exactly why QGC "just works": its joystick stream is accepted, while a stream
# from mavros (source sysid 1 by default) would be silently dropped.
# In 'override' mode we therefore point the autopilot at mavros; in 'default'
# mode we hand authority back to QGC.
MAVROS_SYSID = 1
QGC_SYSID = 255
# The parameter was renamed between ArduPilot versions; we detect which one exists.
GCS_ID_PARAM_CANDIDATES = ["SYSID_MYGCS", "MAV_GCS_SYSID"]

# Watchdog / heartbeat cadence. The watchdog tick also drives the retry
# scheduler, so there is exactly one periodic timer owning recovery.
WATCHDOG_PERIOD_S = 0.5
HEARTBEAT_PERIOD_S = 1.0


class BlueBoatParameterControl(Node):
    def __init__(self):
        super().__init__('blueboat_parameter_control')

        ################## Parameters ##################
        # A cold ParamPull walks the whole ArduPilot parameter table over
        # MAVLink and is genuinely slow on a fresh WiFi link, so the default is
        # generous. It is a give-up bound, not an expected duration.
        self.declare_parameter('param_sequence_timeout_s', 20.0)
        self.declare_parameter('param_retry_limit', 3)
        self.declare_parameter('param_retry_delay_s', 2.0)

        self.sequence_timeout = float(
            self.get_parameter('param_sequence_timeout_s').value)
        self.retry_limit = int(self.get_parameter('param_retry_limit').value)
        self.retry_delay = float(self.get_parameter('param_retry_delay_s').value)

        ################## ROS2 Communication ##################
        # publishers
        self.ready_pub = self.create_publisher(Bool, '/blueboat/param_ready', 10)
        self.mode_pub = self.create_publisher(String, '/blueboat/param_mode', 10)

        # subscriber
        self.sub = self.create_subscription(String, '/blueboat/param_str', self.callback, 10)

        # services
        self.pull_client = self.create_client(ParamPull, '/mavros/param/pull')
        self.get_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.set_client = self.create_client(SetParameters, '/mavros/param/set_parameters')

        # state
        self.params_ready = False
        self.current_mode = None
        self.pending_mode = None
        self.busy = False              # a set/verify sequence is in flight
        self.gcs_id_param = None       # resolved once ("SYSID_MYGCS" or "MAV_GCS_SYSID")

        self._targets = []             # list of (param_name, value) for the pending mode
        self._set_index = 0

        # Recovery state (rules 1 and 2 in the module docstring).
        self._seq = 0                  # generation counter; bumped per attempt
        self._busy_since = 0.0         # monotonic stamp of the in-flight attempt
        self._attempt = 0              # attempts spent on _retry_mode
        self._retry_mode = None        # mode to re-attempt, None when idle
        self._retry_at = 0.0           # monotonic deadline for that re-attempt
        self._warned_services = False

        # NOTE: no blocking wait_for_service loop in the constructor anymore.
        # The old version spun here until mavros was up, so if mavros was slow the
        # node was completely deaf and the (single) 'override' request from
        # robot_interface was lost -> random launch hangs. Service availability is
        # now checked lazily when a request arrives; robot_interface re-sends its
        # request every second until confirmed, so nothing is lost.
        self.watchdog_timer = self.create_timer(WATCHDOG_PERIOD_S, self._watchdog)
        self.heartbeat_timer = self.create_timer(HEARTBEAT_PERIOD_S, self._heartbeat)

    ################## Request handling ##################

    def callback(self, msg: String):
        mode = msg.data.strip()

        # Idempotent: robot_interface re-requests until it hears back
        if mode == self.current_mode:
            self._retry_mode = None
            self.publish_state()
            return

        if self.busy:
            # A sequence is already running (likely for this very mode, since the
            # requester retries). Ignore instead of restarting the pull each time.
            # Bounded by the watchdog: `busy` cannot outlive sequence_timeout.
            self.get_logger().info(
                f"Parameter sequence in progress, ignoring request '{mode}' for now",
                throttle_duration_sec=2.0)
            return

        # An explicit new request supersedes any pending internal retry.
        self._attempt = 0
        self._retry_mode = None
        self.apply_mode(mode)

    def apply_mode(self, mode):
        if mode == "override":
            # Route both thrusters to RC passthrough so robot_interface can stream
            # PWM on /mavros/rc/override (fire-and-forget, no ACK round-trips),
            # and make the autopilot listen to mavros as its GCS.
            servo_targets = [("SERVO1_FUNCTION", RCIN1_PASSTHROUGH),
                             ("SERVO3_FUNCTION", RCIN3_PASSTHROUGH)]
            gcs_target = MAVROS_SYSID
        elif mode == "default":
            servo_targets = [("SERVO1_FUNCTION", THROTTLE_RIGHT),
                             ("SERVO3_FUNCTION", THROTTLE_LEFT)]
            gcs_target = QGC_SYSID
        else:
            # Not a dead end any more: report state so the requester can see that
            # nothing changed rather than waiting on an echo that will never come.
            self.get_logger().error(f"Unknown mode: {mode}")
            self.publish_state()
            return

        # Make sure mavros services exist before starting (non-blocking check)
        for cli in [self.pull_client, self.get_client, self.set_client]:
            if not cli.service_is_ready():
                if not self._warned_services:
                    self._warned_services = True
                    self.get_logger().warn(
                        "mavros parameter services not available yet, will retry on next request")
                self._schedule_retry(mode)
                return
        self._warned_services = False

        self.busy = True
        self._seq += 1
        self._busy_since = time.monotonic()
        self.pending_mode = mode
        self._gcs_target = gcs_target
        self._servo_targets = servo_targets

        self.pull_params(self._seq)

    ################## Recovery: watchdog, generation fence, retry ##################

    def _stale(self, seq):
        """True when `seq` belongs to a sequence that has been abandoned.

        Rule 2: a future completing after its sequence timed out must not write
        state that now belongs to a newer attempt.
        """
        return seq != self._seq

    def _watchdog(self):
        """The only thing that can rescue a sequence whose future never fires."""
        now = time.monotonic()

        if self.busy and (now - self._busy_since) > self.sequence_timeout:
            self.get_logger().error(
                f"Parameter sequence for '{self.pending_mode}' timed out after "
                f"{self.sequence_timeout:.1f} s (mavros unresponsive) - abandoning "
                "so later requests are accepted.")
            self._fail_locked(f"timeout after {self.sequence_timeout:.1f} s")
            return

        if (not self.busy and self._retry_mode is not None
                and now >= self._retry_at):
            mode = self._retry_mode
            self._retry_mode = None
            self.get_logger().info(
                f"Retrying parameter mode '{mode}' "
                f"(attempt {self._attempt + 1}/{self.retry_limit})")
            self.apply_mode(mode)

    def _schedule_retry(self, mode):
        """Queue one more attempt at `mode`, or give up loudly.

        The limit bounds SELF-driven retries only. An external request resets
        the counter (`callback`), which is deliberate: robot_interface re-asks
        every second, and the node must stay willing to try again when the link
        comes back. What the limit buys is that a node nobody is talking to
        stops hammering mavros on its own.
        """
        self._attempt += 1
        if self._attempt >= self.retry_limit:
            self.get_logger().error(
                f"Giving up on parameter mode '{mode}' after {self._attempt} "
                "attempts. The node stays responsive: send the request again "
                "(robot_interface does so every second) once mavros is healthy.")
            self._attempt = 0
            self._retry_mode = None
            self.publish_state()
            return
        self._retry_mode = mode
        self._retry_at = time.monotonic() + self.retry_delay

    def _heartbeat(self):
        """Repeat the current mode so a late subscriber does not wait for a
        transition that may never come. Inert downstream: robot_interface's
        mode_callback is edge-triggered."""
        self.publish_state()

    ################## Sequence: pull -> resolve GCS param -> set all -> verify ##################

    def pull_params(self, seq):
        req = ParamPull.Request()
        future = self.pull_client.call_async(req)
        future.add_done_callback(lambda f: self._on_pull_done(f, seq))

    def _on_pull_done(self, future, seq):
        if self._stale(seq):
            return
        try:
            if not future.result().success:
                raise RuntimeError("Param pull failed")
        except Exception as e:
            self._fail(str(e))
            return

        if self.gcs_id_param is None:
            self._resolve_gcs_param(seq)
        else:
            self._build_targets_and_set(seq)

    def _resolve_gcs_param(self, seq):
        """Ask mavros for both candidate names; the one that exists wins."""
        future = self._get_param_async(GCS_ID_PARAM_CANDIDATES)
        future.add_done_callback(lambda f: self._on_gcs_resolved(f, seq))

    def _on_gcs_resolved(self, future, seq):
        if self._stale(seq):
            return
        try:
            values = future.result().values
            for name, value in zip(GCS_ID_PARAM_CANDIDATES, values):
                if value.type != 0:  # PARAMETER_NOT_SET
                    self.gcs_id_param = name
                    break
        except Exception as e:
            self.get_logger().warn(f"Could not resolve GCS sysid parameter: {e}")

        if self.gcs_id_param is None:
            # Very old/odd firmware: continue with servo params only. RC override
            # will then only work if the autopilot's GCS sysid already matches
            # mavros - log loudly so this is diagnosable in the field.
            self.get_logger().error("Neither SYSID_MYGCS nor MAV_GCS_SYSID found; "
                                    "RC override may be ignored by the autopilot")
        else:
            self.get_logger().info(f"Using GCS sysid parameter '{self.gcs_id_param}'")

        self._build_targets_and_set(seq)

    def _build_targets_and_set(self, seq):
        self._targets = list(self._servo_targets)
        if self.gcs_id_param is not None:
            self._targets.append((self.gcs_id_param, self._gcs_target))

        self._set_index = 0
        self._set_next(seq)

    def _set_next(self, seq):
        if self._set_index >= len(self._targets):
            self._verify(seq)
            return

        name, value = self._targets[self._set_index]
        future = self._set_param_async(name, value)
        future.add_done_callback(lambda f: self._on_set_done(f, seq))

    def _on_set_done(self, future, seq):
        if self._stale(seq):
            return
        name, _ = self._targets[self._set_index]
        try:
            if not future.result().results[0].successful:
                raise RuntimeError(f"Failed to set {name}")
        except Exception as e:
            self._fail(str(e))
            return

        self._set_index += 1
        self._set_next(seq)

    def _verify(self, seq):
        names = [name for name, _ in self._targets]
        future = self._get_param_async(names)
        future.add_done_callback(lambda f: self._on_verify_done(f, seq))

    def _on_verify_done(self, future, seq):
        if self._stale(seq):
            return
        success = False
        try:
            values = future.result().values

            success = True
            for (name, target), value in zip(self._targets, values):
                actual = value.integer_value if value.type == 2 else int(value.double_value)
                if actual != target:
                    self.get_logger().error(f"Verification failed: {name} = {actual}, expected {target}")
                    success = False

            self.params_ready = success
            if success:
                self.current_mode = self.pending_mode
                self._attempt = 0
                self._retry_mode = None
                self.get_logger().info(f"Mode '{self.current_mode}' applied and verified")

        except Exception as e:
            self.get_logger().error(str(e))
            self.params_ready = False

        self.busy = False
        if not success:
            # A verified-wrong autopilot is the partial-application case: some
            # targets may have landed and some not. Re-running the whole
            # sequence is the rollback.
            self._schedule_retry(self.pending_mode)
        self.publish_state()

    def _fail(self, reason):
        """Abandon the in-flight attempt and queue a retry."""
        self.get_logger().error(reason)
        self._fail_locked(reason)

    def _fail_locked(self, reason):
        self._seq += 1            # fence off any future still pending (rule 2)
        self.params_ready = False
        self.busy = False
        mode = self.pending_mode
        self.publish_state()
        if mode is not None:
            self._schedule_retry(mode)

    ################## mavros helpers ##################

    def _set_param_async(self, name, value):
        param = Parameter()
        param.name = name

        val = ParameterValue()
        val.type = 2  # integer
        val.integer_value = int(value)

        param.value = val

        req = SetParameters.Request()
        req.parameters = [param]

        return self.set_client.call_async(req)

    def _get_param_async(self, names):
        req = GetParameters.Request()
        req.names = names
        return self.get_client.call_async(req)

    def publish_state(self):
        ready_msg = Bool()
        ready_msg.data = self.params_ready
        self.ready_pub.publish(ready_msg)

        # Always publish, even before the first successful apply. The empty
        # string means "this node is alive and no mode is locked" - the old
        # `if self.current_mode is not None` made that state indistinguishable
        # from a dead node, and robot_interface logged `current: ''` forever.
        mode_msg = String()
        mode_msg.data = self.current_mode if self.current_mode is not None else ""
        self.mode_pub.publish(mode_msg)

    ################## Shutdown ##################

    def restore_default_blocking(self, budget_s=3.0):
        """Hand the thrusters back to the stock mapping before this node dies.

        CM-15 / ctrlCLAUDE.md N5: killing the stack while SERVO1/3_FUNCTION are
        still on RC passthrough leaves the boat listening to whatever is on RC
        channels 1 and 3, with the mavros that was pinning them gone. Bounded
        so teardown cannot hang - a best-effort restore beats no restore, and
        an operator 'default' before shutdown remains the reliable route.
        """
        if self.current_mode == "default":
            return True
        if not rclpy.ok():
            # The context is already torn down (a KeyboardInterrupt that
            # propagated out of spin can leave it that way): spinning here
            # would raise inside a finally block.
            return False
        if not self.set_client.service_is_ready():
            self.get_logger().warn(
                "Cannot restore the default servo mapping: mavros parameter "
                "service is gone. Send 'default' on /blueboat/input_str once "
                "mavros is back.")
            return False

        targets = [("SERVO1_FUNCTION", THROTTLE_RIGHT),
                   ("SERVO3_FUNCTION", THROTTLE_LEFT)]
        if self.gcs_id_param is not None:
            targets.append((self.gcs_id_param, QGC_SYSID))

        deadline = time.monotonic() + budget_s
        ok = True
        for name, value in targets:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                ok = False
                break
            try:
                future = self._set_param_async(name, value)
                rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
            except Exception:                          # noqa: BLE001 - teardown
                ok = False
                break
            if not future.done():
                ok = False
                break

        if ok:
            self.current_mode = "default"
            self.publish_state()
            self.get_logger().info("Default servo mapping restored on shutdown")
        else:
            self.get_logger().error(
                "Default servo mapping only partially restored on shutdown - "
                "check SERVO1_FUNCTION / SERVO3_FUNCTION in QGroundControl")
        return ok


def main():
    rclpy.init()
    node = BlueBoatParameterControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave the autopilot in RC passthrough with nobody streaming.
        try:
            node.restore_default_blocking()
        except Exception as exc:                       # noqa: BLE001 - teardown
            node.get_logger().error(f"Shutdown restore failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
