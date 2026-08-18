#!/usr/bin/env python3

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


class BlueBoatParameterControl(Node):
    def __init__(self):
        super().__init__('blueboat_parameter_control')

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

        # NOTE: no blocking wait_for_service loop in the constructor anymore.
        # The old version spun here until mavros was up, so if mavros was slow the
        # node was completely deaf and the (single) 'override' request from
        # robot_interface was lost -> random launch hangs. Service availability is
        # now checked lazily when a request arrives; robot_interface re-sends its
        # request every second until confirmed, so nothing is lost.

    ################## Request handling ##################

    def callback(self, msg: String):
        mode = msg.data.strip()

        # Idempotent: robot_interface re-requests until it hears back
        if mode == self.current_mode:
            self.publish_state()
            return

        if self.busy:
            # A sequence is already running (likely for this very mode, since the
            # requester retries). Ignore instead of restarting the pull each time.
            self.get_logger().info(f"Parameter sequence in progress, ignoring request '{mode}' for now")
            return

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
            self.get_logger().error(f"Unknown mode: {mode}")
            return

        # Make sure mavros services exist before starting (non-blocking check)
        for cli in [self.pull_client, self.get_client, self.set_client]:
            if not cli.service_is_ready():
                self.get_logger().warn("mavros parameter services not available yet, will retry on next request")
                return

        self.busy = True
        self.pending_mode = mode
        self._gcs_target = gcs_target
        self._servo_targets = servo_targets

        self.pull_params()

    ################## Sequence: pull -> resolve GCS param -> set all -> verify ##################

    def pull_params(self):
        req = ParamPull.Request()
        future = self.pull_client.call_async(req)
        future.add_done_callback(self._on_pull_done)

    def _on_pull_done(self, future):
        try:
            if not future.result().success:
                raise RuntimeError("Param pull failed")
        except Exception as e:
            self._fail(str(e))
            return

        if self.gcs_id_param is None:
            self._resolve_gcs_param()
        else:
            self._build_targets_and_set()

    def _resolve_gcs_param(self):
        """Ask mavros for both candidate names; the one that exists wins."""
        future = self._get_param_async(GCS_ID_PARAM_CANDIDATES)
        future.add_done_callback(self._on_gcs_resolved)

    def _on_gcs_resolved(self, future):
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

        self._build_targets_and_set()

    def _build_targets_and_set(self):
        self._targets = list(self._servo_targets)
        if self.gcs_id_param is not None:
            self._targets.append((self.gcs_id_param, self._gcs_target))

        self._set_index = 0
        self._set_next()

    def _set_next(self):
        if self._set_index >= len(self._targets):
            self._verify()
            return

        name, value = self._targets[self._set_index]
        future = self._set_param_async(name, value)
        future.add_done_callback(self._on_set_done)

    def _on_set_done(self, future):
        name, _ = self._targets[self._set_index]
        try:
            if not future.result().results[0].successful:
                raise RuntimeError(f"Failed to set {name}")
        except Exception as e:
            self._fail(str(e))
            return

        self._set_index += 1
        self._set_next()

    def _verify(self):
        names = [name for name, _ in self._targets]
        future = self._get_param_async(names)
        future.add_done_callback(self._on_verify_done)

    def _on_verify_done(self, future):
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
                self.get_logger().info(f"Mode '{self.current_mode}' applied and verified")

        except Exception as e:
            self.get_logger().error(str(e))
            self.params_ready = False

        self.busy = False
        self.publish_state()

    def _fail(self, reason):
        self.get_logger().error(reason)
        self.params_ready = False
        self.busy = False
        self.publish_state()

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

        if self.current_mode is not None:
            mode_msg = String()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)


rclpy.init()
node = BlueBoatParameterControl()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
