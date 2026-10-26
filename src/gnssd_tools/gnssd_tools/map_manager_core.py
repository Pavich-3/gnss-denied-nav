"""
The ROS-free parts of map_manager_node (unit-tested in test/test_map_manager_core.py).

* MapStore      - names and files of saved maps in one folder;
* validate_mode_command - checks an operator's set_mode request;
* ChildProcess  - starts and cleanly stops one supervised process
                  (the slam_toolbox launch of the current mode).

Map files. One saved map <name> consists of
    <name>.posegraph, <name>.data   slam_toolbox pose graph (needed to localize later)
    <name>.pgm, <name>.yaml         occupancy grid image (for people and other tools)
"""

import os
import re
import signal
import subprocess
import time

MAPPING = 'mapping'
LOCALIZATION = 'localization'
MODES = (MAPPING, LOCALIZATION)
LAST_CHECKPOINT = 'last_checkpoint'
CHECKPOINT_PREFIX = 'checkpoint_'

NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
POSE_GRAPH_SUFFIXES = ('.posegraph', '.data')
GRID_SUFFIXES = ('.pgm', '.yaml')


class MapStore:
    """Saved maps in one folder."""

    def __init__(self, folder):
        self.folder = os.path.abspath(folder)

    @staticmethod
    def is_valid_name(name):
        """Only letters, digits, '_' and '-': no paths, no surprises in file names."""
        return bool(NAME_PATTERN.match(name or ''))

    def base_path(self, name):
        """Path without extension, as slam_toolbox's serialize/deserialize expect it."""
        if not self.is_valid_name(name):
            raise ValueError(f'Invalid map name "{name}" (letters, digits, _ and - only)')
        return os.path.join(self.folder, name)

    def pose_graph_files(self, name):
        return [self.base_path(name) + suffix for suffix in POSE_GRAPH_SUFFIXES]

    def grid_files(self, name):
        return [self.base_path(name) + suffix for suffix in GRID_SUFFIXES]

    def has_pose_graph(self, name):
        """Tell whether the files needed to localize on map `name` exist."""
        return self.is_valid_name(name) and all(
            os.path.isfile(path) for path in self.pose_graph_files(name))

    def latest_checkpoint(self):
        """Name of the newest checkpoint_* pose graph in the folder, or None."""
        if not os.path.isdir(self.folder):
            return None
        names = [f[:-len('.posegraph')] for f in os.listdir(self.folder)
                 if f.startswith(CHECKPOINT_PREFIX) and f.endswith('.posegraph')]
        names = [n for n in names if self.has_pose_graph(n)]
        return max(names, key=lambda n: os.path.getmtime(self.base_path(n) + '.posegraph'),
                   default=None)


def validate_mode_command(mode, map_name, use_base_pose, base_pose_available, store):
    """
    Check a set_mode request.

    Returns (ok, message, resolved_map_name). Localization needs a map;
    mapping takes one optionally (then it continues that graph). A given map
    must exist (LAST_CHECKPOINT is resolved to the newest checkpoint), and
    use_base_pose needs a known last valid pose.
    """
    if mode not in MODES:
        return False, f'Unknown mode "{mode}" (use "mapping" or "localization")', None
    if mode == MAPPING and not map_name:
        return True, 'mapping (new map)', ''
    if map_name == LAST_CHECKPOINT:
        map_name = store.latest_checkpoint()
        if map_name is None:
            return False, 'No checkpoint has been saved yet', None
    if not store.has_pose_graph(map_name):
        return (False, f'Map "{map_name}" not found in {store.folder} '
                       '(need .posegraph and .data)', None)
    if use_base_pose and not base_pose_available:
        return False, 'No valid corrected pose known yet, give x/y/yaw explicitly', None
    if mode == MAPPING:
        return True, f'mapping, continuing {map_name}', map_name
    return True, f'localization on {map_name}', map_name


class ChildProcess:
    """One supervised process in its own process group (so its children stop with it)."""

    def __init__(self, stop_timeout=10.0):
        self.stop_timeout = stop_timeout
        self.process = None
        self.started_at = None

    def start(self, command, log_file=None):
        if self.is_running():
            raise RuntimeError('A child process is already running')
        output = open(log_file, 'a') if log_file else subprocess.DEVNULL
        self.process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                        start_new_session=True)   # own process group
        self.started_at = time.monotonic()
        return self.process.pid

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    @property
    def pid(self):
        return self.process.pid if self.is_running() else 0

    def uptime(self):
        return time.monotonic() - self.started_at if self.is_running() else 0.0

    def stop(self):
        """
        Stop the whole process group: SIGINT, then SIGTERM, then SIGKILL.

        Returns the signal that finally stopped it ('none' if nothing ran).
        Every step waits up to stop_timeout for the group to exit.
        """
        if not self.is_running():
            self.process = None
            return 'none'
        group = os.getpgid(self.process.pid)
        for sig, name in ((signal.SIGINT, 'SIGINT'), (signal.SIGTERM, 'SIGTERM'),
                          (signal.SIGKILL, 'SIGKILL')):
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                break
            if self._group_gone(group):
                self.process.wait()
                self.process = None
                return name
        self.process = None
        return 'SIGKILL'

    def _group_gone(self, group):
        """Wait up to stop_timeout until no process of the group is left (children included)."""
        deadline = time.monotonic() + self.stop_timeout
        while time.monotonic() < deadline:
            # Reap our direct child once it has exited; an unreaped (zombie)
            # process would still count as a member of the group.
            self.process.poll()
            try:
                os.killpg(group, 0)      # signal 0 = "does anybody in the group exist?"
            except ProcessLookupError:
                return True
            time.sleep(0.1)
        return False
