#!/usr/bin/env python3
"""Standalone host cockpit embedding Gazebo, RViz and the camera view.

The rendering processes stay in their existing x86 containers. On an X11
session, Qt adopts each foreign client window into one normal top-level cockpit
through QWindow.fromWinId/createWindowContainer. The desktop session remains
owned by GNOME/Mutter; this application neither replaces nor reconfigures it.

Manual controls are forwarded to a small ROS publisher inside the viz
container. Commands are sent only while a button/key is held, and zero is sent
on release, focus loss, bridge failure and application shutdown.
"""

from __future__ import annotations

import argparse
import ctypes
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


WINDOW_LINE = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+"([^"]*)"(.*)$')
CLIENT_ID = re.compile(r'0x[0-9a-fA-F]+')
WINDOW_NAME = re.compile(
    r'^(?:_NET_WM_NAME|WM_NAME)\([^)]*\) = "(.*)"$',
    re.MULTILINE,
)
MANUAL_COMMANDS = {
    'forward': (0.25, 0.0, 0.0),
    'backward': (-0.20, 0.0, 0.0),
    'left': (0.0, 0.0, 0.20),
    'right': (0.0, 0.0, -0.20),
}


@dataclass(frozen=True)
class XWindow:
    """One X11 window reported by xwininfo."""

    window_id: int
    title: str
    metadata: str

    @property
    def searchable(self) -> str:
        """Lower-case title plus WM_CLASS-like metadata."""
        return f'{self.title} {self.metadata}'.lower()


def parse_xwininfo(output: str) -> list[XWindow]:
    """Parse the useful rows from xwininfo -root -tree output."""
    windows = []
    for line in output.splitlines():
        match = WINDOW_LINE.match(line)
        if not match:
            continue
        windows.append(XWindow(
            window_id=int(match.group(1), 16),
            title=match.group(2),
            metadata=match.group(3),
        ))
    return windows


PANEL_PATTERNS = {
    'camera': re.compile(r'rqt_image_view|image view|/demo/camera/image_raw'),
    'rviz': re.compile(r'\brviz(?:2)?\b'),
    'simulation': re.compile(r'gazebo|\bgz sim\b|\bgz_sim\b'),
}

CLIENT_CLASSES = {
    'camera': 'rqt_image_view',
    'rviz': 'rviz2',
    'simulation': 'gz-sim-gui',
}


def select_windows(windows: Iterable[XWindow]) -> dict[str, XWindow]:
    """Choose the real client window for every embedded panel."""
    selected = {}
    used = set()
    for panel in ('camera', 'rviz', 'simulation'):
        pattern = PANEL_PATTERNS[panel]
        candidates = [
            window for window in windows
            if window.window_id not in used
            and pattern.search(window.searchable)
            and 'selection owner' not in window.searchable
            and window.title
        ]
        if candidates:
            client_class = CLIENT_CLASSES[panel]
            choice = max(
                candidates,
                key=lambda item: (
                    client_class in item.metadata.lower(),
                    'mutter-x11-frames' not in item.metadata.lower(),
                    len(item.title),
                ),
            )
            selected[panel] = choice
            used.add(choice.window_id)
    return selected


def list_x_windows() -> list[XWindow]:
    """Read only current EWMH top-level clients, excluding stale descendants."""
    client_list = subprocess.run(
        ['xprop', '-root', '_NET_CLIENT_LIST'],
        check=True,
        capture_output=True,
        text=True,
        timeout=3,
    )
    windows = []
    for raw_id in CLIENT_ID.findall(client_list.stdout):
        properties = subprocess.run(
            [
                'xprop', '-id', raw_id,
                '_NET_WM_NAME', 'WM_NAME', 'WM_CLASS',
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=1,
        ).stdout
        names = WINDOW_NAME.findall(properties)
        if not names:
            continue
        windows.append(XWindow(
            window_id=int(raw_id, 16),
            title=names[0],
            metadata=properties,
        ))
    return windows


def teleop_process_command(compose_file: Path) -> list[str]:
    """Build the persistent bridge command used by the manual controls."""
    return [
        'docker', 'compose', '-f', str(compose_file),
        'exec', '-T', 'viz', 'bash', '-lc',
        '. /opt/ros/jazzy/setup.bash; '
        '. /ws/install/setup.bash; '
        'exec python3 -u /cockpit/cockpit_teleop.py',
    ]


class X11Embedder:
    """Reparent and resize foreign clients inside native cockpit surfaces."""

    def __init__(self):
        self.x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XReparentWindow.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong,
            ctypes.c_int, ctypes.c_int,
        ]
        self.x11.XMapWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XMoveResizeWindow.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
            ctypes.c_uint, ctypes.c_uint,
        ]
        self.x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('não foi possível abrir o DISPLAY X11')

    def embed(self, child: int, parent: int) -> None:
        """Move one existing X11 client under a native Qt host widget."""
        self.x11.XReparentWindow(self.display, child, parent, 0, 0)
        self.x11.XMapWindow(self.display, child)
        self.x11.XSync(self.display, False)

    def resize(self, child: int, width: int, height: int) -> None:
        """Fill the host surface with the embedded client."""
        self.x11.XMoveResizeWindow(
            self.display, child, 0, 0,
            max(1, width), max(1, height),
        )
        self.x11.XSync(self.display, False)

    def close(self) -> None:
        """Release this connection; application cleanup stops the clients."""
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None


class TeleopBridge:
    """Persistent stdin bridge to the ROS publisher in the viz container."""

    def __init__(self, compose_file: Path, log_file: Path):
        self.compose_file = compose_file
        self.log_file = log_file
        self.process: subprocess.Popen[str] | None = None
        self._log_handle = None

    def start(self) -> None:
        """Start the bridge once; fail visibly if its container is unavailable."""
        if self.process is not None and self.process.poll() is None:
            return
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        self._log_handle = self.log_file.open('a', encoding='utf-8')
        self.process = subprocess.Popen(
            teleop_process_command(self.compose_file),
            stdin=subprocess.PIPE,
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def send(self, command: tuple[float, float, float]) -> None:
        """Send one command sample; the remote publisher owns the 10 Hz tick."""
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('ponte ROS do controle manual não está ativa')
        assert self.process.stdin is not None
        vx, vy, wz = command
        self.process.stdin.write(f'CMD {vx:.3f} {vy:.3f} {wz:.3f}\n')
        self.process.stdin.flush()

    def stop_motion(self) -> None:
        """Request zero velocity without tearing down the bridge."""
        if self.process is None or self.process.poll() is not None:
            return
        assert self.process.stdin is not None
        try:
            self.process.stdin.write('STOP\n')
            self.process.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    def close(self) -> None:
        """Stop motion first, then terminate the helper deterministically."""
        process = self.process
        if process is not None and process.poll() is None:
            self.stop_motion()
            assert process.stdin is not None
            try:
                process.stdin.write('QUIT\n')
                process.stdin.flush()
                process.stdin.close()
                process.wait(timeout=3)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        self.process = None
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None


def self_test() -> int:
    """Exercise parsing and command construction without X11 or ROS."""
    fixture = """
     0x04a00004 "Qt Selection Owner for rviz2": () 1x1+0+0
     0x04600007 "Gazebo Sim": ("gz-sim-gui" "Gazebo GUI") 1280x900+0+0
     0x04a0000b "RViz": ("rviz2" "rviz2") 1200x700+0+0
     0x05000004 "Qt Selection Owner for rqt_image_view": () 1x1+0+0
     0x05000011 "/demo/camera/image_raw - rqt_image_view":
       ("rqt_image_view" "rqt_image_view") 640x480+0+0
    """
    selected = select_windows(parse_xwininfo(fixture))
    assert selected['simulation'].window_id == int('0x04600007', 16)
    assert selected['rviz'].window_id == int('0x04a0000b', 16)
    assert selected['camera'].window_id == int('0x05000011', 16)
    command = teleop_process_command(Path('/repo/docker/compose.host.yml'))
    assert command[-1].endswith('/cockpit/cockpit_teleop.py')
    assert MANUAL_COMMANDS['forward'] == (0.25, 0.0, 0.0)
    print('cockpit standalone contract: OK')
    return 0


def build_ui(args: argparse.Namespace) -> int:
    """Create one window containing three foreign views and robot controls."""
    try:
        from PyQt5.QtCore import QEvent, QTimer, Qt
        from PyQt5.QtGui import QGuiApplication
        from PyQt5.QtWidgets import (
            QApplication,
            QFrame,
            QGridLayout,
            QHBoxLayout,
            QLabel,
            QPushButton,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as error:
        print(
            'cockpit: PyQt5 ausente no host. Instale python3-pyqt5.',
            file=sys.stderr,
        )
        print(f'cockpit: {error}', file=sys.stderr)
        return 2

    class EmbeddedPanel(QFrame):
        """One titled slot that adopts a foreign X11 client."""

        def __init__(self, title: str, waiting: str, embedder: X11Embedder):
            super().__init__()
            self.setObjectName('panel')
            self.embedder = embedder
            self.child_window_id = None
            layout = QVBoxLayout(self)
            layout.setContentsMargins(1, 1, 1, 1)
            layout.setSpacing(0)
            heading = QLabel(title)
            heading.setObjectName('panelTitle')
            heading.setFixedHeight(30)
            heading.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
            layout.addWidget(heading)
            self.content = QVBoxLayout()
            self.content.setContentsMargins(0, 0, 0, 0)
            self.placeholder = QLabel(waiting)
            self.placeholder.setObjectName('placeholder')
            self.placeholder.setAlignment(Qt.AlignCenter)
            self.content.addWidget(self.placeholder)
            layout.addLayout(self.content, 1)

        def attach(self, candidate: XWindow) -> None:
            """Adopt a foreign client once, preserving its rendering process."""
            if self.child_window_id is not None:
                return
            native_host = QWidget(self)
            native_host.setAttribute(Qt.WA_NativeWindow, True)
            native_host.setFocusPolicy(Qt.StrongFocus)
            host_id = int(native_host.winId())
            self.content.removeWidget(self.placeholder)
            self.placeholder.deleteLater()
            self.content.addWidget(native_host)
            self.native_host = native_host
            self.child_window_id = candidate.window_id
            self.embedder.embed(candidate.window_id, host_id)
            QTimer.singleShot(0, self.resize_embedded)

        def resize_embedded(self) -> None:
            if self.child_window_id is None:
                return
            self.embedder.resize(
                self.child_window_id,
                self.native_host.width(),
                self.native_host.height(),
            )

        def resizeEvent(self, event) -> None:
            super().resizeEvent(event)
            if self.child_window_id is not None:
                QTimer.singleShot(0, self.resize_embedded)

    class CockpitWindow(QWidget):
        """Single operator window containing visualization and manual control."""

        KEY_COMMANDS = {
            Qt.Key_W: 'forward',
            Qt.Key_Up: 'forward',
            Qt.Key_S: 'backward',
            Qt.Key_Down: 'backward',
            Qt.Key_A: 'left',
            Qt.Key_Left: 'left',
            Qt.Key_D: 'right',
            Qt.Key_Right: 'right',
        }

        def __init__(self):
            super().__init__()
            self.setWindowTitle(f'Aquila AM69 — Cockpit {args.mode.upper()}')
            self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
            self.setFocusPolicy(Qt.StrongFocus)
            self.active_command = None
            self.panels_attached = set()
            self.embedder = X11Embedder()
            self.teleop = TeleopBridge(
                Path(args.compose_file),
                Path(args.teleop_log),
            )

            root = QVBoxLayout(self)
            root.setContentsMargins(6, 6, 6, 6)
            root.setSpacing(6)
            root.addLayout(self._make_header())

            views = QHBoxLayout()
            views.setSpacing(6)
            self.panels = {
                'simulation': EmbeddedPanel(
                    'SIMULAÇÃO 3D — GAZEBO', 'Aguardando Gazebo…',
                    self.embedder,
                ),
                'rviz': EmbeddedPanel(
                    'NAVEGAÇÃO — RVIZ', 'Aguardando RViz…', self.embedder,
                ),
                'camera': EmbeddedPanel(
                    'CÂMERA — /demo/camera/image_raw',
                    'Aguardando câmera…',
                    self.embedder,
                ),
            }
            views.addWidget(self.panels['simulation'], 3)
            right = QVBoxLayout()
            right.setSpacing(6)
            right.addWidget(self.panels['rviz'], 3)
            right.addWidget(self.panels['camera'], 2)
            views.addLayout(right, 2)
            root.addLayout(views, 1)
            root.addWidget(self._make_controls())

            self.setStyleSheet("""
                QWidget { background: #10151b; color: #e8edf2; }
                QFrame#panel {
                    background: #080b0f; border: 1px solid #34495a;
                }
                QLabel#title {
                    color: #65c7f7; font-weight: 800; font-size: 17px;
                }
                QLabel#status { color: #9fb4c5; }
                QLabel#panelTitle {
                    background: #1b2530; color: #a9d9ef;
                    font-weight: 700; padding-left: 9px;
                }
                QLabel#placeholder { color: #617383; font-size: 14px; }
                QFrame#controls {
                    background: #17202a; border: 1px solid #34495a;
                }
                QPushButton {
                    background: #24597a; border: 1px solid #4f8fb5;
                    border-radius: 4px; padding: 7px 14px;
                    font-weight: 700;
                }
                QPushButton:pressed { background: #53a9d1; color: #071018; }
                QPushButton#stop {
                    background: #8a2633; border-color: #d45c68;
                    min-width: 120px; min-height: 52px;
                }
                QPushButton#close {
                    background: #71343b; border-color: #a6535e;
                }
            """)

            self.discovery_timer = QTimer(self)
            self.discovery_timer.timeout.connect(self.discover_and_embed)
            self.discovery_timer.start(500)
            self.command_timer = QTimer(self)
            self.command_timer.timeout.connect(self.repeat_manual_command)
            self.command_timer.setInterval(150)

            try:
                self.teleop.start()
                self.control_status.setText('Controle manual conectado')
            except OSError as error:
                self.control_status.setText(f'Controle indisponível: {error}')

        def _make_header(self):
            header = QHBoxLayout()
            title = QLabel(
                f'AQUILA AM69  |  COCKPIT STANDALONE  |  {args.mode.upper()}'
            )
            title.setObjectName('title')
            header.addWidget(title)
            header.addStretch(1)
            self.status = QLabel('Aguardando 3 visualizações…')
            self.status.setObjectName('status')
            header.addWidget(self.status)
            retry = QPushButton('Reconectar painéis')
            retry.setFocusPolicy(Qt.NoFocus)
            retry.clicked.connect(self.discover_and_embed)
            header.addWidget(retry)
            close = QPushButton('Encerrar')
            close.setObjectName('close')
            close.setFocusPolicy(Qt.NoFocus)
            close.clicked.connect(self.close)
            header.addWidget(close)
            return header

        def _make_controls(self):
            frame = QFrame()
            frame.setObjectName('controls')
            row = QHBoxLayout(frame)
            row.setContentsMargins(12, 6, 12, 6)
            legend = QVBoxLayout()
            label = QLabel('CONTROLE MANUAL')
            label.setStyleSheet('font-weight: 800; color: #65c7f7;')
            legend.addWidget(label)
            legend.addWidget(QLabel(
                'Segure W/S/A/D ou as setas. Use sem uma meta Nav2 ativa.'
            ))
            self.control_status = QLabel('Conectando controle…')
            self.control_status.setObjectName('status')
            legend.addWidget(self.control_status)
            row.addLayout(legend)
            row.addStretch(1)

            buttons = QGridLayout()
            controls = (
                ('forward', 'W / ↑\nFrente', 0, 1),
                ('left', 'A / ←\nGirar esquerda', 1, 0),
                ('backward', 'S / ↓\nRé', 1, 1),
                ('right', 'D / →\nGirar direita', 1, 2),
            )
            for name, text, grid_row, column in controls:
                button = QPushButton(text)
                button.setFocusPolicy(Qt.NoFocus)
                button.pressed.connect(
                    lambda command=name: self.begin_manual(command)
                )
                button.released.connect(self.end_manual)
                buttons.addWidget(button, grid_row, column)
            row.addLayout(buttons)

            stop = QPushButton('PARAR\nESPAÇO')
            stop.setObjectName('stop')
            stop.setFocusPolicy(Qt.NoFocus)
            stop.clicked.connect(self.end_manual)
            row.addWidget(stop)
            return frame

        def show_on_requested_screen(self) -> None:
            screens = QGuiApplication.screens()
            screen = QGuiApplication.primaryScreen()
            if args.screen != 'primary':
                screen = next(
                    (candidate for candidate in screens
                     if candidate.name() == args.screen),
                    screen,
                )
            if screen is not None:
                self.setGeometry(screen.availableGeometry())
            self.show()
            self.raise_()
            self.activateWindow()

        def discover_and_embed(self) -> None:
            try:
                candidates = select_windows(list_x_windows())
            except (
                OSError, RuntimeError, subprocess.SubprocessError,
            ) as error:
                self.status.setText(f'X11 indisponível: {error}')
                return

            # Attach atomically. Once an OpenGL client such as Gazebo has been
            # adopted, walking the entire root tree can become very expensive.
            # Waiting until all three IDs are known avoids needing another
            # discovery pass through a partially embedded hierarchy.
            found = len(candidates)
            if found < 3:
                self.status.setText(f'{found}/3 visualizações encontradas')
                return

            try:
                for name in ('simulation', 'rviz', 'camera'):
                    self.panels[name].attach(candidates[name])
                    self.panels_attached.add(name)
            except RuntimeError as error:
                self.status.setText(f'Falha ao incorporar painel: {error}')
                return

            self.status.setText('3/3 visualizações incorporadas')
            self.discovery_timer.stop()
            self.raise_()

        def begin_manual(self, name: str) -> None:
            self.active_command = MANUAL_COMMANDS[name]
            self.repeat_manual_command()
            self.command_timer.start()

        def repeat_manual_command(self) -> None:
            if self.active_command is None:
                return
            try:
                self.teleop.send(self.active_command)
                vx, _, wz = self.active_command
                self.control_status.setText(
                    f'Comando ativo: vx={vx:+.2f}, wz={wz:+.2f}'
                )
            except (BrokenPipeError, OSError, RuntimeError) as error:
                self.end_manual()
                self.control_status.setText(f'Controle indisponível: {error}')

        def end_manual(self) -> None:
            self.active_command = None
            self.command_timer.stop()
            self.teleop.stop_motion()
            self.control_status.setText('Controle manual parado')

        def keyPressEvent(self, event) -> None:
            if event.key() == Qt.Key_Space:
                self.end_manual()
                return
            command = self.KEY_COMMANDS.get(event.key())
            if command is not None and not event.isAutoRepeat():
                self.begin_manual(command)
                return
            super().keyPressEvent(event)

        def keyReleaseEvent(self, event) -> None:
            if (
                event.key() in self.KEY_COMMANDS
                and not event.isAutoRepeat()
            ):
                self.end_manual()
                return
            super().keyReleaseEvent(event)

        def changeEvent(self, event) -> None:
            if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
                self.end_manual()
            super().changeEvent(event)

        def closeEvent(self, event) -> None:
            self.end_manual()
            self.teleop.close()
            self.embedder.close()
            event.accept()

    app = QApplication(sys.argv[:1])
    app.setApplicationName('Aquila Cockpit')
    window = CockpitWindow()
    window.show_on_requested_screen()
    QTimer.singleShot(250, window.discover_and_embed)
    return app.exec_()


def parse_args() -> argparse.Namespace:
    """Parse cockpit arguments; service orchestration remains in Bash."""
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--screen', default='primary',
        help='Qt/XRandR screen name (default: primary, e.g. DP-1 or eDP-1).',
    )
    parser.add_argument(
        '--mode', choices=('hil', 'learn'), default='hil',
        help='Label the active execution mode in the cockpit title.',
    )
    parser.add_argument(
        '--compose-file',
        default=str(repo_root / 'docker' / 'compose.host.yml'),
        help='Host Compose file used to open the persistent teleop bridge.',
    )
    parser.add_argument(
        '--teleop-log',
        default=str(repo_root / 'log' / 'cockpit' / 'teleop.log'),
        help='Log file for the ROS manual-control helper.',
    )
    parser.add_argument(
        '--self-test', action='store_true',
        help='Test parsing and command construction without opening the UI.',
    )
    return parser.parse_args()


def main() -> int:
    """CLI entrypoint."""
    args = parse_args()
    if args.self_test:
        return self_test()
    return build_ui(args)


if __name__ == '__main__':
    raise SystemExit(main())
