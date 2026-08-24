"""Regression tests for the host cockpit's desktop-session boundary."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
COCKPIT = REPO_ROOT / 'scripts' / 'cockpit.py'
TELEOP = REPO_ROOT / 'scripts' / 'cockpit_teleop.py'
LAUNCHER = REPO_ROOT / 'scripts' / 'run_cockpit.sh'

# None of these operations belongs in an application launcher. Keep this list
# literal so a future implementation that tries one fails before bench use.
FORBIDDEN_DESKTOP_OPERATIONS = (
    'gnome-shell',
    'gnome-extensions',
    'gsettings',
    '--replace',
    'XKillClient',
    'systemctl --user',
)


def load_cockpit_module():
    """Load the host-only script without requiring ROS or a display."""
    spec = importlib.util.spec_from_file_location('aquila_cockpit', COCKPIT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_launcher_cannot_manage_the_desktop_session():
    """Keep session replacement, reconfiguration and reparenting out."""
    implementation = COCKPIT.read_text() + LAUNCHER.read_text()
    for operation in FORBIDDEN_DESKTOP_OPERATIONS:
        assert operation not in implementation


def test_controller_is_a_normal_top_level_window():
    """Prevent Mutter from withdrawing an unowned Qt utility window."""
    implementation = COCKPIT.read_text()
    assert 'setWindowFlags(Qt.Window | Qt.FramelessWindowHint)' in implementation
    assert '| Qt.Tool' not in implementation


def test_standalone_window_embeds_all_three_clients():
    """The cockpit must contain views, not merely arrange top-level windows."""
    implementation = COCKPIT.read_text()
    assert 'class X11Embedder:' in implementation
    assert 'self.x11.XReparentWindow' in implementation
    assert 'self.x11.XMoveResizeWindow' in implementation
    assert 'Qt.WA_NativeWindow' in implementation
    assert "['xprop', '-root', '_NET_CLIENT_LIST']" in implementation
    assert "['xwininfo', '-root', '-tree']" not in implementation
    assert "'simulation': EmbeddedPanel" in implementation
    assert "'rviz': EmbeddedPanel" in implementation
    assert "'camera': EmbeddedPanel" in implementation
    assert 'if found < 3:' in implementation
    assert "for name in ('simulation', 'rviz', 'camera')" in implementation


def test_manual_control_has_deadman_and_zero_paths():
    """Every motion path must expire and understand an explicit stop."""
    cockpit = load_cockpit_module()
    assert all(
        abs(value) <= 0.5
        for command in cockpit.MANUAL_COMMANDS.values()
        for value in command
    )
    helper = TELEOP.read_text()
    assert "fields == ['STOP']" in helper
    assert "fields == ['QUIT']" in helper
    assert 'COMMAND_TIMEOUT = 0.40' in helper
    assert "make_twist((0.0, 0.0, 0.0))" in helper


def test_window_selection_ignores_qt_helpers():
    """Select real clients while leaving ownership with the window manager."""
    cockpit = load_cockpit_module()
    fixture = '''
      0x1 "Qt Selection Owner for rviz2": () 1x1+0+0
      0x2 "Gazebo Sim": ("gz-sim-gui" "Gz-sim-gui") 1280x900+0+0
      0x3 "RViz": ("rviz2" "rviz2") 1200x700+0+0
      0x4 "/demo/camera/image_raw - rqt_image_view":
          ("rqt_image_view" "rqt_image_view") 640x480+0+0
    '''
    selected = cockpit.select_windows(cockpit.parse_xwininfo(fixture))
    assert {name: window.window_id for name, window in selected.items()} == {
        'simulation': 0x2,
        'rviz': 0x3,
        'camera': 0x4,
    }
