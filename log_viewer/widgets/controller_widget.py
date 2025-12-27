from __future__ import annotations

from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets


class ControllerLogWidget(QtWidgets.QWidget):
    """Placeholder for controller_log.txt (future work)."""

    def __init__(self, parent: QtWidgets.QWidget | None = None):
        super().__init__(parent)

        self._title = QtWidgets.QLabel("Controller")
        self._title.setObjectName("panelTitle")

        self._path_label = QtWidgets.QLabel("-")
        self._path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        self._text = QtWidgets.QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(self._title)
        outer.addWidget(self._path_label)
        outer.addWidget(self._text, 1)

        self._path: Path | None = None

    def set_file(self, controller_log: Path) -> None:
        self._path = controller_log
        self._path_label.setText(str(controller_log))
        if not controller_log.exists():
            self._text.setPlainText("No controller log found (controller_log.txt).\n")
            return

        try:
            # For now: just show the first N lines to avoid huge loads.
            lines = controller_log.read_text(errors="replace").splitlines()
            head = "\n".join(lines[:400])
            if len(lines) > 400:
                head += f"\n\n... ({len(lines) - 400} more lines)"
            self._text.setPlainText(head)
        except Exception as e:
            self._text.setPlainText(f"Failed to read controller log: {e}\n")
