"""Unit tests of the ROS-free parts of map_manager_node (gnssd_tools.map_manager_core)."""

import os
import sys
import time

import pytest

from gnssd_tools.map_manager_core import (ChildProcess, LAST_CHECKPOINT, MapStore,
                                          validate_mode_command)


def touch(path):
    with open(path, 'w') as f:
        f.write('x')


def test_map_names_and_paths(tmp_path):
    store = MapStore(str(tmp_path))
    assert store.is_valid_name('room_s1') and store.is_valid_name('wh-2')
    for bad in ('', '../etc', 'a/b', 'name with space', 'x' * 65):
        assert not store.is_valid_name(bad)
    with pytest.raises(ValueError):
        store.base_path('../escape')
    assert store.pose_graph_files('room_s1') == [str(tmp_path / 'room_s1.posegraph'),
                                                 str(tmp_path / 'room_s1.data')]


def test_pose_graph_needs_both_files(tmp_path):
    store = MapStore(str(tmp_path))
    touch(tmp_path / 'room_s1.posegraph')
    assert not store.has_pose_graph('room_s1')
    touch(tmp_path / 'room_s1.data')
    assert store.has_pose_graph('room_s1')


def test_latest_checkpoint(tmp_path):
    store = MapStore(str(tmp_path))
    assert store.latest_checkpoint() is None
    for name in ('checkpoint_100', 'checkpoint_200'):
        touch(tmp_path / f'{name}.posegraph')
        touch(tmp_path / f'{name}.data')
        time.sleep(0.02)
    assert store.latest_checkpoint() == 'checkpoint_200'


def test_mode_command_validation(tmp_path):
    store = MapStore(str(tmp_path))
    ok, _, _ = validate_mode_command('flying', '', False, True, store)
    assert not ok
    ok, _, name = validate_mode_command('mapping', '', False, False, store)
    assert ok and name == ''
    ok, message, _ = validate_mode_command('mapping', 'missing', False, True, store)
    assert not ok and 'not found' in message
    ok, message, _ = validate_mode_command('localization', 'room_s1', False, True, store)
    assert not ok and 'not found' in message
    touch(tmp_path / 'room_s1.posegraph')
    touch(tmp_path / 'room_s1.data')
    ok, _, name = validate_mode_command('localization', 'room_s1', False, False, store)
    assert ok and name == 'room_s1'
    ok, message, _ = validate_mode_command('localization', 'room_s1', True, False, store)
    assert not ok and 'valid corrected pose' in message
    ok, message, _ = validate_mode_command('localization', LAST_CHECKPOINT, False, True, store)
    assert not ok and 'checkpoint' in message


def test_child_process_stops_cleanly_with_sigint():
    child = ChildProcess(stop_timeout=5.0)
    child.start([sys.executable, '-c', 'import time; time.sleep(60)'])
    assert child.is_running() and child.pid > 0
    assert child.stop() == 'SIGINT'          # KeyboardInterrupt ends the python child
    assert not child.is_running() and child.pid == 0


def test_child_process_escalates_and_leaves_no_orphans():
    child = ChildProcess(stop_timeout=1.0)
    # The child ignores SIGINT and starts a grandchild: both must be gone afterwards.
    script = ('import signal, subprocess, time, sys; '
              'signal.signal(signal.SIGINT, signal.SIG_IGN); '
              'subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"]); '
              'time.sleep(60)')
    child.start([sys.executable, '-c', script])
    time.sleep(0.5)
    group = os.getpgid(child.pid)
    assert child.stop() in ('SIGTERM', 'SIGKILL')
    with pytest.raises(ProcessLookupError):
        os.killpg(group, 0)


def test_mapping_can_continue_a_saved_graph(tmp_path):
    store = MapStore(str(tmp_path))
    for name in ('checkpoint_100', 'checkpoint_200'):
        touch(tmp_path / f'{name}.posegraph')
        touch(tmp_path / f'{name}.data')
        time.sleep(0.02)
    ok, message, name = validate_mode_command('mapping', LAST_CHECKPOINT, True, True, store)
    assert ok and name == 'checkpoint_200' and 'continuing' in message
    ok, _, _ = validate_mode_command('mapping', LAST_CHECKPOINT, True, False, store)
    assert not ok   # use_base_pose without a known pose
