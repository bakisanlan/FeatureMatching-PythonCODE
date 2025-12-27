## FeatureMatch Log Viewer

A small PyQt-based GUI to browse per-run logs under `logs/<run_ts>/`.

### What it shows

- **ImageMatch**: browse `logs/<run>/ImageMatch/measurement_update_*.png` with left/right arrows.
- **Trajectory**: plot N×3 `.npy` arrays from `logs/<run>/traj/` with 2D/3D toggle and series checkboxes.
- **Terminal**: view `terminal_pf.log` / `terminal_*.log` with ANSI colors.
- **Controller**: placeholder for `controller_log.txt` (shows first lines).

### Run

From repo root:

```bash
python -m log_viewer
```

Optionally pass a custom logs directory:

```bash
python -m log_viewer /path/to/logs
```

### Notes

- The terminal viewer uses a small ANSI→HTML converter to keep things fast and dependency-light.
- The trajectory plot is embedded Matplotlib (non-blocking) rather than calling `plotter.visualizeTraj()` directly.
