"""Small logging helpers.

Goal: console-only logging that still looks good in the terminal.

- No per-process log files.
- Safe to call from many scripts (won't add duplicate handlers).
- Optional colored output if `colorlog` is installed.

Note on colors + tee:
- ANSI colors are preserved when piping to `tee` only if the program keeps emitting
  ANSI escapes when stdout isn't a TTY. This helper can force that.
"""

from __future__ import annotations

import atexit
import glob
import logging
import os
import sys
import hashlib
from datetime import datetime
from pathlib import Path
from typing import Optional


_DEFAULT_FORMAT = "%(asctime)s [%(name)s] %(levelname)s: %(message)s"
_DEFAULT_DATEFMT = "%H:%M:%S"


_LOGGING_INITIALIZED = False


def _merge_ros_logs_at_exit() -> None:
    """Merge ROS2 rcutils logs into unified log file at process exit.

    If RCUTILS_LOG_FILE_PATH is set, ROS2 rcutils writes per-process log files under
    ~/.ros/log/... which are only fully flushed at exit.

    This handler appends that content to the unified log file.
    """
    unified_log = os.environ.get("UNIFIED_LOG_FILE")
    if not unified_log:
        return

    pid = os.getpid()
    patterns = [
        os.path.expanduser(f"~/.ros/log/python_{pid}_*.log"),
        os.path.expanduser(f"~/.ros/log/python3_{pid}_*.log"),
    ]

    ros_log_files: list[str] = []
    for pattern in patterns:
        ros_log_files.extend(glob.glob(pattern))

    if not ros_log_files:
        return

    ros_log_files.sort(key=lambda p: os.path.getmtime(p))
    ros_log_path = ros_log_files[-1]

    try:
        with open(ros_log_path, "r", encoding="utf-8", errors="replace") as src:
            ros_content = src.read()

        if ros_content:
            with open(unified_log, "a", encoding="utf-8") as dst:
                dst.write("\n" + "=" * 80 + "\n")
                dst.write(f"ROS2 Logs (from {os.path.basename(ros_log_path)})\n")
                dst.write("=" * 80 + "\n")
                dst.write(ros_content)
                if not ros_content.endswith("\n"):
                    dst.write("\n")
    except Exception:
        # We're at exit time; avoid raising.
        pass


def attach_ros_logger_to_python_logging(
    node: object,
    logger_name: str | None = None,
) -> None:
    """Backward-compatible shim.

    Historically the repo experimented with bridging rclpy logs into python logging.
    We now capture rcutils logs via merge-at-exit when file logging is enabled.
    In console_only mode, we intentionally avoid rcutils log files.
    """
    _ = (node, logger_name)
    return


def setup_unified_logging(
    log_file: str | os.PathLike | None = None,
    log_dir_name: str = "logs_out",
    log_file_prefix: str = "unified",
    level: int = logging.INFO,
    force: bool = False,
    console_only: bool = False,
    force_color: bool | None = None,
):
    """Unified colored console logging (optionally unified file), safe for multi-script runs.

    Key properties we rely on in this repo:
    - Console-only mode creates **no log files at all** (tee is the persistence layer).
    - Colors are by *logger name* (stable palette).
    - Idempotent: safe to call from multiple scripts without stacking handlers.
    - When level=DEBUG, DEBUG from any module in this repo should show.

    Returns:
        Path | None: unified log path in file mode, else None.
    """
    global _LOGGING_INITIALIZED

    if _LOGGING_INITIALIZED and not force:
        root = logging.getLogger()
        for h in root.handlers:
            if isinstance(h, logging.FileHandler):
                try:
                    return Path(h.baseFilename)
                except Exception:
                    break
        return None

    # Allow env override so *all* processes/scripts can share one file easily.
    env_log_file = os.environ.get("UNIFIED_LOG_FILE")
    if log_file is None and env_log_file:
        log_file = env_log_file

    # Console-only mode: do NOT create any log file.
    if console_only:
        log_file = None
        # Prevent ROS2 rcutils from writing per-process log files.
        os.environ.pop("RCUTILS_LOG_FILE_PATH", None)

    # Allow env override for log verbosity.
    env_log_level = os.environ.get("LOG_LEVEL")
    if env_log_level:
        env_log_level = env_log_level.strip()
        try:
            if env_log_level.isdigit():
                level = int(env_log_level)
            else:
                level = getattr(logging, env_log_level.upper())
        except Exception:
            pass

    quiet_third_party = os.environ.get("QUIET_THIRD_PARTY_LOGS", "1") not in {"0", "false", "False"}
    third_party_level_name = os.environ.get("THIRD_PARTY_LOG_LEVEL", "INFO").strip()
    try:
        third_party_level = int(third_party_level_name) if third_party_level_name.isdigit() else getattr(logging, third_party_level_name.upper())
    except Exception:
        third_party_level = logging.INFO
    third_party_logger_csv = os.environ.get(
        "THIRD_PARTY_LOGGERS",
        "PIL,PIL.Image,matplotlib,matplotlib.font_manager,urllib3,asyncio,h5py,numba",
    )
    third_party_loggers = [s.strip() for s in third_party_logger_csv.split(",") if s.strip()]

    def _discover_project_logger_prefixes() -> tuple[str, ...]:
        prefixes: set[str] = set()
        try:
            root_dir = Path(os.getcwd())
            if not root_dir.exists():
                return tuple()

            for p in root_dir.glob("*.py"):
                if p.name.startswith("."):
                    continue
                prefixes.add(p.stem)

            for d in root_dir.iterdir():
                if not d.is_dir() or d.name.startswith("."):
                    continue
                try:
                    if any(d.glob("*.py")) or any(d.rglob("__init__.py")):
                        prefixes.add(d.name)
                except Exception:
                    continue
        except Exception:
            return tuple()

        prefixes = {p for p in prefixes if p and p.replace("_", "").isalnum()}
        return tuple(sorted(prefixes))

    project_prefixes = _discover_project_logger_prefixes()

    log_path: Path | None
    if log_file is None:
        log_path = None
    else:
        log_path = Path(log_file)
        if log_path.is_dir():
            log_path = log_path / f"{log_file_prefix}_{datetime.now():%Y%m%d_%H%M%S}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)

    # --- ROS2 / rcutils logging integration ---
    os.environ.setdefault("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}")
    if log_path is not None:
        os.environ.setdefault("RCUTILS_LOGGING_USE_STDOUT", "0")
        os.environ["RCUTILS_LOG_FILE_PATH"] = str(log_path)

    # We'll color only the logger name segment, not the whole line.
    LOG_FORMAT = "%(asctime)s [%(process)d:%(threadName)s] %(levelname)-8s %(name)s: %(message)s"

    # Console coloring (per logger name)
    if force_color is None:
        enable_color = os.environ.get("NO_COLOR", "0") not in {"1", "true", "True"}
    else:
        enable_color = bool(force_color)

    ansi_reset = "\033[0m"
    ansi_by_color = {
        "black": "\033[30m",
        "red": "\033[31m",
        "green": "\033[32m",
        "yellow": "\033[33m",
        "blue": "\033[34m",
        "magenta": "\033[35m",
        "cyan": "\033[36m",
        "white": "\033[37m",
        "bright_black": "\033[90m",
        "bright_red": "\033[91m",
        "bright_green": "\033[92m",
        "bright_yellow": "\033[93m",
        "bright_blue": "\033[94m",
        "bright_magenta": "\033[95m",
        "bright_cyan": "\033[96m",
        "bright_white": "\033[97m",
    }

    palette = [
        "cyan",
        "green",
        "yellow",
        "magenta",
        "blue",
        "bright_cyan",
        "bright_green",
        "bright_yellow",
        "bright_magenta",
        "bright_blue",
    ]

    def pick_color(key: str) -> str:
        """Pick a stable palette color derived from a string key.

        We intentionally use the *full* key (not only the top-level logger group)
        to reduce collisions where multiple modules end up with the same color.
        """
        if not key:
            return "white"
        idx = sum(ord(c) for c in key) % len(palette)
        return palette[idx]

    def _pick_color_from_key(key: str) -> str:
        """Pick a stable palette color derived from an arbitrary string.

        This is used for STDOUT/STDERR redirected output where the effective
        logger name is always "STDOUT"/"STDERR" and we'd otherwise lose
        per-module coloring.
        """
        if not key:
            return "white"
        digest = hashlib.md5(key.encode("utf-8"), usedforsecurity=False).digest()  # stable
        idx = digest[0] % len(palette)
        return palette[idx]

    def _color_key_for_record(record: logging.LogRecord) -> str:
        name = getattr(record, "name", "")
        if name in {"STDOUT", "STDERR"}:
            path = getattr(record, "pathname", "") or ""
            base = os.path.basename(path)
            stem, _ext = os.path.splitext(base)
            return stem or name
        return name

    class ColorByLoggerNameFormatter(logging.Formatter):
        def __init__(self, fmt: str, *, enable: bool):
            super().__init__(fmt)
            self._enable = enable

        def format(self, record: logging.LogRecord) -> str:
            line = super().format(record)
            if not self._enable:
                return line

            # Choose a stable color key (special handling for redirected STDOUT/STDERR).
            key = _color_key_for_record(record)
            if getattr(record, "name", "") in {"STDOUT", "STDERR"}:
                color_key = _pick_color_from_key(key)
            else:
                color_key = pick_color(key)

            prefix = ansi_by_color.get(color_key, "")
            if not prefix:
                return line
            return f"{prefix}{line}{ansi_reset}"

    root = logging.getLogger()
    if root.handlers:
        for h in list(root.handlers):
            root.removeHandler(h)

    file_handler: logging.Handler | None = None
    if log_path is not None:
        file_handler = logging.FileHandler(log_path)
        file_handler.setLevel(level)
        file_handler.setFormatter(logging.Formatter(LOG_FORMAT))

    console_handler = logging.StreamHandler(sys.__stdout__)
    console_handler.setLevel(level)
    console_handler.setFormatter(ColorByLoggerNameFormatter(LOG_FORMAT, enable=enable_color))

    root.setLevel(level)
    if file_handler is not None:
        root.addHandler(file_handler)
    root.addHandler(console_handler)

    for pfx in project_prefixes:
        try:
            logging.getLogger(pfx).setLevel(logging.NOTSET)
        except Exception:
            pass

    if quiet_third_party and level <= logging.DEBUG:
        for name in third_party_loggers:
            if any(name == p or name.startswith(p + ".") for p in project_prefixes):
                continue
            logging.getLogger(name).setLevel(third_party_level)

    class _StreamToLogger:
        def __init__(self, logger_obj: logging.Logger, lvl: int):
            self.logger = logger_obj
            self.level = lvl

        def write(self, buf: str) -> None:
            buf = buf.rstrip()
            if not buf:
                return
            for line in buf.splitlines():
                # Increase stacklevel so the LogRecord points to the original caller
                # (helps per-file coloring for STDOUT/STDERR and improves traceability).
                try:
                    self.logger.log(self.level, line.rstrip(), stacklevel=3)
                except TypeError:
                    # Older logging implementations might not support stacklevel.
                    self.logger.log(self.level, line.rstrip())

        def flush(self) -> None:
            return

    sys.stdout = _StreamToLogger(logging.getLogger("STDOUT"), logging.INFO)
    sys.stderr = _StreamToLogger(logging.getLogger("STDERR"), logging.ERROR)

    _LOGGING_INITIALIZED = True

    if log_path is not None:
        logging.getLogger(__name__).info("Unified logging initialized: %s", str(log_path))
        os.environ["UNIFIED_LOG_FILE"] = str(log_path)
        atexit.register(_merge_ros_logs_at_exit)
        return log_path

    logging.getLogger(__name__).info("Unified logging initialized (console-only)")
    return None


def _truthy_env(name: str) -> bool:
    val = os.environ.get(name)
    if val is None:
        return False
    return val.strip().lower() not in {"0", "false", "no", "off", ""}


def setup_console_logging(
    *,
    level: int = logging.INFO,
    fmt: str = _DEFAULT_FORMAT,
    datefmt: str = _DEFAULT_DATEFMT,
    force_color: Optional[bool] = None,
    root: bool = True,
) -> None:
    """Configure console logging (optionally colored) exactly once.

    Args:
        level: Root logger level.
        fmt: Log message format.
        datefmt: Date format.
        force_color:
            - True: always emit ANSI colors (useful with `tee -a`, CI, etc.)
            - False: never emit colors
            - None: auto (use TTY detection)
        root: If True, configure the *root* logger. If False, configure only this
            module's logger; generally keep True for apps.

    Environment variables:
        FEATUREMATCH_LOG_FORCE_COLOR=1  -> forces colors
        FEATUREMATCH_LOG_NO_COLOR=1     -> disables colors

    This function is idempotent: calling it multiple times won't stack handlers.
    """

    if force_color is None:
        if _truthy_env("FEATUREMATCH_LOG_NO_COLOR"):
            force_color = False
        elif _truthy_env("FEATUREMATCH_LOG_FORCE_COLOR"):
            force_color = True

    use_color = False
    if force_color is True:
        use_color = True
    elif force_color is False:
        use_color = False
    else:
        # Auto: enable color only when writing to an interactive terminal.
        use_color = hasattr(sys.stderr, "isatty") and sys.stderr.isatty()

    target_logger = logging.getLogger() if root else logging.getLogger(__name__)

    # Idempotency: if we've already installed our handler, just update level.
    for h in list(target_logger.handlers):
        if getattr(h, "_featurematch_console_handler", False):
            target_logger.setLevel(level)
            return

    target_logger.setLevel(level)

    handler: logging.Handler
    if use_color:
        try:
            import colorlog  # type: ignore

            cfmt = (
                "%(log_color)s%(asctime)s [%(name)s] %(levelname)s:%(reset)s "
                "%(message)s"
            )
            formatter = colorlog.ColoredFormatter(
                cfmt,
                datefmt=datefmt,
                reset=True,
                log_colors={
                    "DEBUG": "cyan",
                    "INFO": "green",
                    "WARNING": "yellow",
                    "ERROR": "red",
                    "CRITICAL": "bold_red",
                },
            )
            handler = logging.StreamHandler(stream=sys.stderr)
            handler.setFormatter(formatter)
        except Exception:
            # Fall back to non-colored formatting if colorlog isn't installed.
            handler = logging.StreamHandler(stream=sys.stderr)
            handler.setFormatter(logging.Formatter(fmt=fmt, datefmt=datefmt))
    else:
        handler = logging.StreamHandler(stream=sys.stderr)
        handler.setFormatter(logging.Formatter(fmt=fmt, datefmt=datefmt))

    # Mark to avoid duplication.
    handler._featurematch_console_handler = True  # type: ignore[attr-defined]
    target_logger.addHandler(handler)

    # Reduce noisy libraries a bit (safe defaults; adjust anytime).
    logging.getLogger("matplotlib").setLevel(max(level, logging.WARNING))
    logging.getLogger("PIL").setLevel(max(level, logging.WARNING))
