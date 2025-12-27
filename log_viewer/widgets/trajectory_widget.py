from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

# Add parent directory to path for utils import
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from utils import quat2eul


class TrajectoryWidget(QtWidgets.QWidget):
    """Embedded trajectory plotting with Position, Velocity, Orientation, and All States views."""

    # Plot mode constants
    MODE_POSITION = "Position"
    MODE_VELOCITY = "Velocity"
    MODE_ORIENTATION = "Orientation"
    MODE_ALL_STATES = "All States"

    def __init__(self, parent: QtWidgets.QWidget | None = None):
        super().__init__(parent)

        self._traj_dir: Path | None = None
        self._data: dict[str, np.ndarray] = {}

        self._title = QtWidgets.QLabel("Trajectory")
        self._title.setObjectName("panelTitle")

        self._path_label = QtWidgets.QLabel("-")
        self._path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        # Plot mode selector
        self._mode_combo = QtWidgets.QComboBox()
        self._mode_combo.addItems([
            self.MODE_POSITION,
            self.MODE_VELOCITY,
            self.MODE_ORIENTATION,
            self.MODE_ALL_STATES,
        ])

        self._plot3d = QtWidgets.QCheckBox("3D")
        self._plot3d.setChecked(True)

        # Line vs Scatter toggle
        self._use_lines = QtWidgets.QCheckBox("Lines")
        self._use_lines.setChecked(True)
        self._use_lines.setToolTip("Use line plots (faster) instead of scatter plots")

        # Series toggles
        self._show_vio = QtWidgets.QCheckBox("VIO")
        self._show_pf = QtWidgets.QCheckBox("PF")
        self._show_gt = QtWidgets.QCheckBox("GT")
        self._show_ref = QtWidgets.QCheckBox("Generated")
        for cb in (self._show_vio, self._show_pf, self._show_gt, self._show_ref):
            cb.setChecked(True)

        self._refresh_btn = QtWidgets.QPushButton("Refresh")

        # Controls layout
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Mode:"))
        controls.addWidget(self._mode_combo)
        controls.addSpacing(10)
        controls.addWidget(self._plot3d)
        controls.addWidget(self._use_lines)
        controls.addSpacing(10)
        controls.addWidget(self._show_vio)
        controls.addWidget(self._show_pf)
        controls.addWidget(self._show_gt)
        controls.addWidget(self._show_ref)
        controls.addStretch(1)
        controls.addWidget(self._refresh_btn)

        # Larger figure for better 3D visualization
        self._fig = Figure(figsize=(8, 6), dpi=100)
        self._canvas = FigureCanvas(self._fig)
        self._toolbar = NavigationToolbar(self._canvas, self)
        self._toolbar.setIconSize(QtCore.QSize(18, 18))

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(self._title)
        outer.addWidget(self._path_label)
        outer.addLayout(controls)
        outer.addWidget(self._toolbar)
        outer.addWidget(self._canvas, 1)

        # Connect signals
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        for w in (self._plot3d, self._use_lines, self._show_vio, self._show_pf, self._show_gt, self._show_ref):
            w.stateChanged.connect(self.redraw)
        self._refresh_btn.clicked.connect(self.reload_and_redraw)

    def _on_mode_changed(self, _idx: int) -> None:
        """Handle plot mode change."""
        mode = self._mode_combo.currentText()
        # 3D checkbox only applies to Position mode
        self._plot3d.setEnabled(mode == self.MODE_POSITION)
        # Generated trajectory only in Position mode
        self._show_ref.setEnabled(mode == self.MODE_POSITION)
        self.redraw()

    def set_directory(self, traj_dir: Path) -> None:
        self._traj_dir = traj_dir
        self._path_label.setText(str(traj_dir))
        self.reload_and_redraw()

    def reload_and_redraw(self) -> None:
        self._data = {}
        if not self._traj_dir or not self._traj_dir.exists():
            self.redraw()
            return

        # Load all available data files
        data_files = [
            # Position
            "VIO_pos_list.npy", "PF_pos_list.npy", "GT_pos_list.npy", "generated_traj.npy",
            # Velocity
            "VIO_vel_list.npy", "GT_vel_list.npy",
            # Orientation
            "VIO_ori_list.npy", "PF_ori_list.npy", "GT_ori_list.npy",
            # Timestamps
            "VIO_ts_list.npy", "PF_ts_list.npy", "GT_ts_list.npy",
        ]
        
        for name in data_files:
            p = self._traj_dir / name
            if p.exists():
                try:
                    self._data[name] = np.load(p)
                except Exception:
                    pass

        self.redraw()

    def redraw(self) -> None:
        self._fig.clear()
        mode = self._mode_combo.currentText()

        # Check if we have any data
        if not self._data:
            self._draw_no_data_message()
            return

        if mode == self.MODE_POSITION:
            self._draw_position_plot()
        elif mode == self.MODE_VELOCITY:
            self._draw_velocity_plot()
        elif mode == self.MODE_ORIENTATION:
            self._draw_orientation_plot()
        elif mode == self.MODE_ALL_STATES:
            self._draw_all_states_plot()

        self._canvas.draw_idle()

    def _draw_no_data_message(self) -> None:
        ax = self._fig.add_subplot(111)
        ax.axis("off")
        if self._traj_dir is None:
            msg = "No trajectory directory selected."
        else:
            msg = (
                "No trajectory data found.\n\n"
                "Expected files under traj/:\n"
                "- VIO_pos_list.npy, VIO_vel_list.npy, VIO_ori_list.npy\n"
                "- GT_pos_list.npy, GT_vel_list.npy, GT_ori_list.npy\n"
                "- PF_pos_list.npy, PF_ori_list.npy"
            )
        ax.text(0.5, 0.5, msg, ha="center", va="center", fontsize=11, wrap=True)
        self._canvas.draw_idle()

    def _draw_position_plot(self) -> None:
        """Draw 2D or 3D position trajectory."""
        plot_3d = self._plot3d.isChecked()
        bounds: dict[str, list[float]] = {"x": [], "y": [], "z": []}

        def accumulate_bounds(arr: np.ndarray | None) -> None:
            if arr is None:
                return
            a = np.asarray(arr)
            if a.ndim != 2 or a.shape[1] < 2:
                return
            bounds["x"].extend([float(np.nanmin(a[:, 1])), float(np.nanmax(a[:, 1]))])
            bounds["y"].extend([float(np.nanmin(a[:, 0])), float(np.nanmax(a[:, 0]))])
            if a.shape[1] >= 3:
                bounds["z"].extend([float(np.nanmin(a[:, 2])), float(np.nanmax(a[:, 2]))])

        if self._show_vio.isChecked():
            accumulate_bounds(self._data.get("VIO_pos_list.npy"))
        if self._show_pf.isChecked():
            accumulate_bounds(self._data.get("PF_pos_list.npy"))
        if self._show_gt.isChecked():
            accumulate_bounds(self._data.get("GT_pos_list.npy"))
        if self._show_ref.isChecked():
            accumulate_bounds(self._data.get("generated_traj.npy"))

        if plot_3d:
            ax = self._fig.add_subplot(111, projection="3d")
            ax.set_xlabel("East")
            ax.set_ylabel("North")
            ax.set_zlabel("Up")
            ax.invert_zaxis()  # Invert Z so positive is up (NED -> ENU for display)
        else:
            ax = self._fig.add_subplot(111)
            ax.set_xlabel("East")
            ax.set_ylabel("North")

        def plot_series(arr: np.ndarray | None, label: str, color: str):
            if arr is None:
                return
            a = np.asarray(arr)
            if a.ndim != 2 or a.shape[1] < 2:
                return
            use_lines = self._use_lines.isChecked()
            if plot_3d:
                if a.shape[1] < 3:
                    return
                if use_lines:
                    ax.plot(a[:, 1], a[:, 0], -a[:, 2], label=label, color=color, linewidth=2.0, alpha=0.9)
                else:
                    ax.scatter(a[:, 1], a[:, 0], -a[:, 2], s=6, alpha=0.9, label=label, color=color)
            else:
                if use_lines:
                    ax.plot(a[:, 1], a[:, 0], label=label, color=color, linewidth=2.0, alpha=0.9)
                else:
                    ax.scatter(a[:, 1], a[:, 0], s=8, alpha=0.9, label=label, color=color)

        if self._show_vio.isChecked():
            plot_series(self._data.get("VIO_pos_list.npy"), "VIO", "C0")
        if self._show_pf.isChecked():
            plot_series(self._data.get("PF_pos_list.npy"), "PF", "C2")
        if self._show_gt.isChecked():
            plot_series(self._data.get("GT_pos_list.npy"), "GT", "C1")
        if self._show_ref.isChecked():
            plot_series(self._data.get("generated_traj.npy"), "Generated", "C3")

        # Equal axis scaling
        if bounds["x"] and bounds["y"]:
            x_min, x_max = min(bounds["x"]), max(bounds["x"])
            y_min, y_max = min(bounds["y"]), max(bounds["y"])
            x_mid, y_mid = 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)
            max_range = max(x_max - x_min, y_max - y_min)

            if plot_3d:
                # Negate Z bounds since we display -Z (Up instead of Down)
                z_vals = bounds["z"] if bounds["z"] else [0.0]
                z_min, z_max = -max(z_vals), -min(z_vals)  # Negate and swap min/max
                z_mid = 0.5 * (z_min + z_max)
                max_range = max(max_range, z_max - z_min)
                half = 0.5 * max_range if max_range > 0 else 1.0
                ax.set_xlim(x_mid - half, x_mid + half)
                ax.set_ylim(y_mid - half, y_mid + half)
                ax.set_zlim(z_mid - half, z_mid + half)
            else:
                half = 0.5 * max_range if max_range > 0 else 1.0
                ax.set_xlim(x_mid - half, x_mid + half)
                ax.set_ylim(y_mid - half, y_mid + half)
                ax.set_aspect("equal", adjustable="box")

        ax.grid(True)
        ax.legend(loc="best")

    def _draw_velocity_plot(self) -> None:
        """Draw 3 subplots for velocity (N, E, D) over time."""
        axes = self._fig.subplots(3, 1, sharex=True)
        labels = ["North [m/s]", "East [m/s]", "Down [m/s]"]

        # Get time vectors and velocity data
        datasets = []
        if self._show_vio.isChecked() and "VIO_vel_list.npy" in self._data:
            ts = self._data.get("VIO_ts_list.npy")
            vel = self._data["VIO_vel_list.npy"]
            if ts is None:
                ts = np.arange(len(vel)) * 0.05  # Assume 20 Hz
            else:
                ts = ts - ts[0]
            datasets.append(("VIO", ts, vel, "C0"))

        if self._show_gt.isChecked() and "GT_vel_list.npy" in self._data:
            ts = self._data.get("GT_ts_list.npy")
            vel = self._data["GT_vel_list.npy"]
            if ts is None:
                ts = np.arange(len(vel)) * 0.05
            else:
                ts = ts - ts[0]
            datasets.append(("GT", ts, vel, "C1"))

        for i, (ax, ylabel) in enumerate(zip(axes, labels)):
            for name, ts, vel, color in datasets:
                if vel.shape[1] > i:
                    ax.plot(ts[:len(vel)], vel[:, i], label=name, color=color, linewidth=0.8)
            ax.set_ylabel(ylabel)
            ax.grid(True)
            if i == 0:
                ax.legend(loc="upper right")

        axes[-1].set_xlabel("Time [s]")
        self._fig.suptitle("Velocity Comparison", fontsize=12)
        self._fig.tight_layout()

    def _draw_orientation_plot(self) -> None:
        """Draw 3 subplots for orientation (Yaw, Pitch, Roll) over time."""
        axes = self._fig.subplots(3, 1, sharex=True)
        labels = ["Yaw [°]", "Pitch [°]", "Roll [°]"]

        datasets = []
        
        # VIO orientation
        if self._show_vio.isChecked() and "VIO_ori_list.npy" in self._data:
            ts = self._data.get("VIO_ts_list.npy")
            ori = self._data["VIO_ori_list.npy"]
            if ts is None:
                ts = np.arange(len(ori)) * 0.05
            else:
                ts = ts - ts[0]
            # Convert quaternion to euler (quat2eul expects [w,x,y,z], returns [yaw, pitch, roll])
            euler = np.rad2deg(quat2eul(ori))  # ori is [qw, qx, qy, qz] or needs reorder
            datasets.append(("VIO", ts, euler, "C0"))

        # GT orientation
        if self._show_gt.isChecked() and "GT_ori_list.npy" in self._data:
            ts = self._data.get("GT_ts_list.npy")
            ori = self._data["GT_ori_list.npy"]
            if ts is None:
                ts = np.arange(len(ori)) * 0.05
            else:
                ts = ts - ts[0]
            euler = np.rad2deg(quat2eul(ori))
            datasets.append(("GT", ts, euler, "C1"))

        # PF orientation
        if self._show_pf.isChecked() and "PF_ori_list.npy" in self._data:
            ts = self._data.get("PF_ts_list.npy")
            ori = self._data["PF_ori_list.npy"]
            if ts is None:
                ts = np.arange(len(ori)) * 0.05
            else:
                ts = ts - ts[0]
            euler = np.rad2deg(quat2eul(ori))
            datasets.append(("PF", ts, euler, "C2"))

        for i, (ax, ylabel) in enumerate(zip(axes, labels)):
            for name, ts, euler, color in datasets:
                if euler.ndim == 2 and euler.shape[1] > i:
                    ax.plot(ts[:len(euler)], euler[:, i], label=name, color=color, linewidth=0.8)
            ax.set_ylabel(ylabel)
            ax.grid(True)
            if i == 0:
                ax.legend(loc="upper right")

        axes[-1].set_xlabel("Time [s]")
        self._fig.suptitle("Orientation Comparison", fontsize=12)
        self._fig.tight_layout()

    def _draw_all_states_plot(self) -> None:
        """Draw 3x3 grid: Position, Velocity, Orientation (rows) for N, E, D / Yaw, Pitch, Roll (cols)."""
        axes = self._fig.subplots(3, 3, sharex="col")
        self._fig.suptitle("VIO vs GT vs PF Comparison", fontsize=12)

        # Get datasets with time alignment
        def get_dataset(prefix: str, color: str, checkbox):
            if not checkbox.isChecked():
                return None
            pos = self._data.get(f"{prefix}_pos_list.npy")
            vel = self._data.get(f"{prefix}_vel_list.npy")
            ori = self._data.get(f"{prefix}_ori_list.npy")
            ts = self._data.get(f"{prefix}_ts_list.npy")
            
            if pos is None:
                return None
                
            if ts is None:
                ts = np.arange(len(pos)) * 0.05
            else:
                ts = ts - ts[0]
            
            euler = None
            if ori is not None:
                try:
                    euler = np.rad2deg(quat2eul(ori))
                except:
                    pass
                    
            return {"name": prefix, "ts": ts, "pos": pos, "vel": vel, "euler": euler, "color": color}

        datasets = [
            d for d in [
                get_dataset("VIO", "C0", self._show_vio),
                get_dataset("GT", "C1", self._show_gt),
                get_dataset("PF", "C2", self._show_pf),
            ] if d is not None
        ]

        # Row 0: Position (N, E, D)
        pos_labels = ["North [m]", "East [m]", "Down [m]"]
        for i, ylabel in enumerate(pos_labels):
            ax = axes[0, i]
            for d in datasets:
                if d["pos"] is not None and d["pos"].shape[1] > i:
                    ax.plot(d["ts"][:len(d["pos"])], d["pos"][:, i], label=d["name"], color=d["color"], linewidth=0.8)
            ax.set_title(ylabel, fontsize=10)
            ax.grid(True)
            if i == 0:
                ax.set_ylabel("Position")
            if i == 1:
                ax.legend(loc="upper right", fontsize=8)

        # Row 1: Velocity (N, E, D)
        vel_labels = ["North [m/s]", "East [m/s]", "Down [m/s]"]
        for i, ylabel in enumerate(vel_labels):
            ax = axes[1, i]
            for d in datasets:
                if d["vel"] is not None and d["vel"].shape[1] > i:
                    ax.plot(d["ts"][:len(d["vel"])], d["vel"][:, i], label=d["name"], color=d["color"], linewidth=0.8)
            ax.set_title(ylabel, fontsize=10)
            ax.grid(True)
            if i == 0:
                ax.set_ylabel("Velocity")

        # Row 2: Euler angles (Yaw, Pitch, Roll)
        euler_labels = ["Yaw [°]", "Pitch [°]", "Roll [°]"]
        for i, ylabel in enumerate(euler_labels):
            ax = axes[2, i]
            for d in datasets:
                if d["euler"] is not None and d["euler"].ndim == 2 and d["euler"].shape[1] > i:
                    ax.plot(d["ts"][:len(d["euler"])], d["euler"][:, i], label=d["name"], color=d["color"], linewidth=0.8)
            ax.set_title(ylabel, fontsize=10)
            ax.grid(True)
            if i == 0:
                ax.set_ylabel("Euler Angles")
            ax.set_xlabel("Time [s]")

        self._fig.tight_layout(rect=[0, 0.03, 1, 0.95])
