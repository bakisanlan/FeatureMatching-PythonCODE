from __future__ import annotations

import sys
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from log_viewer.models import iter_run_dirs, resolve_run_paths
from log_viewer.widgets.controller_widget import ControllerLogWidget
from log_viewer.widgets.image_match_widget import ImageMatchWidget
from log_viewer.widgets.terminal_widget import TerminalLogWidget
from log_viewer.widgets.trajectory_widget import TrajectoryWidget


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, logs_root: Path):
        super().__init__()

        self.setWindowTitle("Visual Navigation Log Viewer")
        self.resize(1400, 900)

        self._logs_root = logs_root

        # --- Run selector ---
        self._run_combo = QtWidgets.QComboBox()
        self._refresh_runs_btn = QtWidgets.QToolButton()
        self._refresh_runs_btn.setText("⟳")
        self._refresh_runs_btn.setToolTip("Refresh run list")

        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Run:"))
        top_bar.addWidget(self._run_combo, 1)
        top_bar.addWidget(self._refresh_runs_btn)

        top_bar_widget = QtWidgets.QWidget()
        top_bar_widget.setLayout(top_bar)

        # --- Panels (tabs) ---
        self._tabs = QtWidgets.QTabWidget()
        self._tabs.setTabPosition(QtWidgets.QTabWidget.North)

        self._image_panel = ImageMatchWidget()
        self._traj_panel = TrajectoryWidget()
        self._terminal_panel = TerminalLogWidget()
        self._controller_panel = ControllerLogWidget()

        self._tabs.addTab(self._image_panel, "ImageMatch")
        self._tabs.addTab(self._traj_panel, "Trajectory")
        self._tabs.addTab(self._terminal_panel, "Terminal")
        self._tabs.addTab(self._controller_panel, "Controller")

        # Layout
        central = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(central)
        outer.addWidget(top_bar_widget)
        outer.addWidget(self._tabs, 1)
        self.setCentralWidget(central)

        self._refresh_runs_btn.clicked.connect(self.refresh_runs)
        self._run_combo.currentIndexChanged.connect(self._on_run_changed)

        self.refresh_runs()

    def refresh_runs(self) -> None:
        run_dirs = list(iter_run_dirs(self._logs_root))
        self._run_combo.blockSignals(True)
        self._run_combo.clear()
        for d in run_dirs:
            self._run_combo.addItem(d.name, str(d))
        self._run_combo.blockSignals(False)

        if run_dirs:
            self._run_combo.setCurrentIndex(0)
            self._load_run(run_dirs[0])
        else:
            self._load_run(None)

    def _on_run_changed(self, idx: int) -> None:
        if idx < 0:
            return
        path_str = self._run_combo.itemData(idx)
        if not path_str:
            return
        self._load_run(Path(path_str))

    def _load_run(self, run_dir: Path | None) -> None:
        if run_dir is None:
            self._image_panel.set_directory(Path("/nonexistent"))
            self._traj_panel.set_directory(Path("/nonexistent"))
            self._terminal_panel.set_file(Path("/nonexistent"))
            self._controller_panel.set_file(Path("/nonexistent"))
            return

        rp = resolve_run_paths(run_dir)
        self._image_panel.set_directory(rp.image_match_dir)
        self._traj_panel.set_directory(rp.traj_dir)
        self._terminal_panel.set_file(rp.terminal_log)
        self._controller_panel.set_file(rp.controller_log)


def apply_theme(app: QtWidgets.QApplication) -> None:
    app.setStyle("Fusion")
    # Lighter "charcoal" theme: still dark-ish, but much less contrast than #121212.
    # Keep it centralized here so tweaking colors is easy.
    app.setStyleSheet(
        """
        QWidget { background-color: #2b2f36; color: #e9edf1; }
        QMainWindow { background-color: #2b2f36; }
        QTabWidget::pane { border: 1px solid #404651; }
        QTabBar::tab { background: #343a44; padding: 8px 12px; border: 1px solid #404651; border-bottom: none; }
        QTabBar::tab:selected { background: #3f4652; }
        QLabel#panelTitle { font-size: 16px; font-weight: 600; padding-top: 6px; }
        QLineEdit, QPlainTextEdit, QTextEdit { background: #262a31; border: 1px solid #404651; selection-background-color: #4a6ea9; }
        QPushButton, QToolButton { background: #343a44; border: 1px solid #404651; padding: 6px 10px; border-radius: 6px; }
        QPushButton:hover, QToolButton:hover { background: #3f4652; }
        QPushButton:pressed, QToolButton:pressed { background: #4a515e; }
        QComboBox { background: #343a44; border: 1px solid #404651; padding: 4px 8px; border-radius: 6px; }
        QComboBox QAbstractItemView { background: #343a44; selection-background-color: #4a6ea9; }
        QScrollBar:vertical { background: #2b2f36; width: 12px; margin: 0; }
        QScrollBar::handle:vertical { background: #4a515e; min-height: 20px; border-radius: 5px; }
        QScrollBar::handle:vertical:hover { background: #5a6271; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
        """
    )


def main(argv: list[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]

    repo_root = Path(__file__).resolve().parents[1]
    logs_root = repo_root / "logs"

    if argv:
        logs_root = Path(argv[0]).expanduser().resolve()

    app = QtWidgets.QApplication(sys.argv)
    apply_theme(app)

    win = MainWindow(logs_root=logs_root)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
