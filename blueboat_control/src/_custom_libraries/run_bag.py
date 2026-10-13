#!/usr/bin/env python3

"""
Per-run ros2bag recorder process (robot_interface, run sessions, v1.1).

One bag per real-robot RUN, filed by poslog_report into the run's
`Robot_data/<stem>/` folder beside the CSV. It carries every topic of the graph
except the side-scan sonar (the .svlog recorder already holds that): all of
/mavros/*, what this stack sends to the boat, the targets, the pinger and the
session plumbing -- enough to replay the run's 3D motion (bag_replay.py).

WHY A SEPARATE PROCESS. The recorder is `ros2 bag record` (rosbag2's C++
writer) in its own OS process, so not one byte of bag data passes through
robot_interface's interpreter: no GIL contention with the 20 Hz RC override
stream. Everything here that can block (waiting for the recorder to close its
file) runs on a worker thread.

WHY PRE-ARMED. `ros2 bag record` needs one to two seconds to start and discover
the topics, which would cut the first seconds of every run. So the next run's
recorder is started PAUSED while the session idles (`--start-paused`); at run
start robot_interface calls its `~/resume` service and recording begins at
once, on subscriptions that already exist.

WHY A STAGING NAME. The run's folder is created by poslog_report at stop, and
only there (write-once: it never touches a folder that already exists), and
the run's name is not known before the run starts. The recorder therefore
writes into a hidden `Robot_data/.run-bag-<pid>-<k>/` and poslog_report MOVES
that directory into the run folder under its final name. Renaming a bag
directory is safe: its metadata.yaml stores the .mcap file names relative to it.

SAME PROCESS GROUP as robot_interface, deliberately (no start_new_session): a
launch teardown SIGINTs the whole group, and SIGINT is exactly what makes
rosbag2 close the file cleanly. robot_interface's shutdown hook then waits for
it, bounded, before filing the run.

ROS-FREE (stdlib only), like poslog_report: callable from a plain prompt.

    >>> import run_bag
    >>> rec = run_bag.RunBagRecorder('/tmp/x/.run-bag-1-1', 'blueboat_run_bag_1')
    >>> rec.start(paused=False); ...; rec.stop()
"""

import os
import shutil
import signal
import subprocess
import sys
import threading

# Topics the bag never records: the side-scan sonar stack (driver, processor,
# AI) -- the .svlog recorder is the sonar's field record.
SSS_EXCLUDE_REGEX = r'^/(side_scan_sonar|sss_processor|sss_ai)(/|$)'

STAGING_PREFIX = '.run-bag-'

_RESET_SIGINT_AND_EXEC = ('import os, signal, sys; '
                          'signal.signal(signal.SIGINT, signal.SIG_DFL); '
                          'os.execvp(sys.argv[1], sys.argv[1:])')

# Escalation when closing a recorder: SIGINT lets rosbag2 flush and write
# metadata.yaml; SIGTERM and SIGKILL are for a hung process only.
SIGINT_WAIT_S = 10.0
SIGTERM_WAIT_S = 5.0


def record_command(output_dir, node_name, paused, storage='mcap',
                   exclude_regex=SSS_EXCLUDE_REGEX):
    """The `ros2 bag record` command line for one run's bag."""
    cmd = ['ros2', 'bag', 'record',
           '--storage', storage,
           '--all-topics',
           '--exclude-regex', exclude_regex,
           '--node-name', node_name,
           # The recorder has no terminal: never let it read keystrokes (a
           # stray space would toggle pause).
           '--disable-keyboard-controls',
           '--output', output_dir]
    if paused:
        cmd.append('--start-paused')
    return cmd


def staging_dir(robot_data_dir, k):
    """A fresh hidden staging path for the k-th recorder of this process."""
    return os.path.join(robot_data_dir, f'{STAGING_PREFIX}{os.getpid()}-{k}')


def bag_has_metadata(path):
    """True once rosbag2 has closed the bag (metadata.yaml is written last)."""
    return os.path.isfile(os.path.join(path, 'metadata.yaml'))


class RunBagRecorder:
    """One `ros2 bag record` process writing one run's bag."""

    def __init__(self, output_dir, node_name, storage='mcap',
                 exclude_regex=SSS_EXCLUDE_REGEX):
        self.output_dir = output_dir
        self.node_name = node_name
        self.storage = storage
        self.exclude_regex = exclude_regex
        self.proc = None
        self.paused = None
        self._stop_lock = threading.Lock()
        self._stopping = False
        self.returncode = None

    # -- lifecycle -------------------------------------------------------

    def start(self, paused):
        """Spawn the recorder. Raises OSError if it cannot be started."""
        cmd = record_command(self.output_dir, self.node_name, paused,
                             self.storage, self.exclude_regex)
        # A parent started in the background of a non-interactive shell
        # inherits SIGINT as IGNORED, and so would the recorder: then the
        # SIGINT that closes the bag cleanly would do nothing. A tiny exec
        # wrapper resets it (the disposition survives exec, the PID is kept).
        # Not preexec_fn: that runs Python in the forked child of a threaded
        # node, and a child stuck before exec would block this Popen -- on
        # robot_interface's executor thread, which streams the RC override.
        self.proc = subprocess.Popen(
            [sys.executable, '-c', _RESET_SIGINT_AND_EXEC, *cmd],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.paused = paused
        return self

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    @property
    def resume_service(self):
        return f'/{self.node_name}/resume'

    def stop(self, sigint_wait=SIGINT_WAIT_S, sigterm_wait=SIGTERM_WAIT_S):
        """Close the recorder, blocking (bounded). Returns its exit code, or
        None if it was never started. Safe to call twice and from two threads:
        the second caller waits for the first."""
        with self._stop_lock:
            if self.proc is None:
                return None
            if self.proc.poll() is None and not self._stopping:
                self._stopping = True
                self._signal(signal.SIGINT)
            try:
                self.returncode = self.proc.wait(timeout=sigint_wait)
            except subprocess.TimeoutExpired:
                self._signal(signal.SIGTERM)
                try:
                    self.returncode = self.proc.wait(timeout=sigterm_wait)
                except subprocess.TimeoutExpired:
                    self._signal(signal.SIGKILL)
                    self.returncode = self.proc.wait()
            return self.returncode

    def stop_async(self, on_closed=None, on_error=None):
        """stop() on a daemon worker thread, then on_closed(self).

        The caller's thread (robot_interface's executor) never waits for the
        recorder to flush."""
        def work():
            try:
                self.stop()
            except Exception as exc:                  # noqa: BLE001 - reported
                if on_error is not None:
                    on_error(self, exc)
                return
            if on_closed is not None:
                on_closed(self)
        thread = threading.Thread(target=work, name=f'{self.node_name}-stop',
                                  daemon=True)
        thread.start()
        return thread

    def discard(self):
        """Stop and delete a staging bag that never recorded a run."""
        self.stop()
        shutil.rmtree(self.output_dir, ignore_errors=True)

    def has_data(self):
        """The bag directory exists and was closed by rosbag2."""
        return bag_has_metadata(self.output_dir)

    # -- internals -------------------------------------------------------

    def _signal(self, sig):
        try:
            self.proc.send_signal(sig)
        except ProcessLookupError:
            pass
