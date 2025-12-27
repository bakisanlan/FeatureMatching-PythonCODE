from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class RunPaths:
    """Resolved paths for a single run folder under `logs/<run_ts>/`."""

    run_dir: Path
    image_match_dir: Path
    traj_dir: Path
    terminal_log: Path
    controller_log: Path


def iter_run_dirs(logs_root: Path) -> Iterable[Path]:
    """Yield run directories under logs_root.

    A run directory is considered valid if it contains at least one of:
      - ImageMatch/
      - traj/
      - terminal_*.log or terminal_pf.log
      - controller_log.txt

    We ignore known archive folders.
    """

    if not logs_root.exists():
        return

    for p in sorted(logs_root.iterdir(), reverse=True):
        if not p.is_dir():
            continue
        if p.name.lower() in {"archieve", "archive", "old_poslist", "logs_out"}:
            continue

        has_any = any(
            (p / name).exists()
            for name in [
                "ImageMatch",
                "traj",
                "controller_log.txt",
                "terminal_pf.log",
                "terminal_vio.log",
                "terminal_pf.txt",
            ]
        )
        if not has_any:
            # Also accept terminal_*.log
            if not any(p.glob("terminal_*.log")):
                continue

        yield p


def resolve_run_paths(run_dir: Path) -> RunPaths:
    """Create a RunPaths object with best-effort filenames."""

    image_match_dir = run_dir / "ImageMatch"
    traj_dir = run_dir / "traj"
    controller_log = run_dir / "controller_log.txt"

    # Prefer the canonical names, but accept terminal_*.log created by the runner.
    terminal_log = run_dir / "terminal_pf.log"
    if not terminal_log.exists():
        # pick first terminal_*.log
        candidates = sorted(run_dir.glob("terminal_*.log"))
        terminal_log = candidates[0] if candidates else terminal_log

    return RunPaths(
        run_dir=run_dir,
        image_match_dir=image_match_dir,
        traj_dir=traj_dir,
        terminal_log=terminal_log,
        controller_log=controller_log,
    )
