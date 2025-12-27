from __future__ import annotations

from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from log_viewer.ansi import ansi_to_html


class TerminalLogWidget(QtWidgets.QWidget):
    """Scrollable terminal log viewer with ANSI color rendering and search."""

    def __init__(self, parent: QtWidgets.QWidget | None = None):
        super().__init__(parent)

        self._title = QtWidgets.QLabel("Terminal")
        self._title.setObjectName("panelTitle")

        self._path_label = QtWidgets.QLabel("-")
        self._path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        self._text = QtWidgets.QTextEdit()
        self._text.setReadOnly(True)
        self._text.setLineWrapMode(QtWidgets.QTextEdit.WidgetWidth)
        self._text.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))

        self._reload_btn = QtWidgets.QPushButton("Reload")

        # --- Search bar ---
        self._search_input = QtWidgets.QLineEdit()
        self._search_input.setPlaceholderText("Search...")
        self._search_input.setClearButtonEnabled(True)
        self._search_input.setMaximumWidth(250)

        self._prev_btn = QtWidgets.QToolButton()
        self._prev_btn.setText("◀")
        self._prev_btn.setToolTip("Previous match (Shift+Enter)")

        self._next_btn = QtWidgets.QToolButton()
        self._next_btn.setText("▶")
        self._next_btn.setToolTip("Next match (Enter)")

        self._match_label = QtWidgets.QLabel("")
        self._match_label.setMinimumWidth(80)

        self._case_sensitive_cb = QtWidgets.QCheckBox("Aa")
        self._case_sensitive_cb.setToolTip("Case sensitive")

        # Search state
        self._search_positions: list[int] = []
        self._current_match_idx: int = -1
        self._original_html: str = ""

        # Layout
        top = QtWidgets.QHBoxLayout()
        top.addWidget(self._reload_btn)
        top.addStretch(1)
        top.addWidget(self._search_input)
        top.addWidget(self._prev_btn)
        top.addWidget(self._next_btn)
        top.addWidget(self._match_label)
        top.addWidget(self._case_sensitive_cb)

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(self._title)
        outer.addWidget(self._path_label)
        outer.addLayout(top)
        outer.addWidget(self._text, 1)

        # Connections
        self._reload_btn.clicked.connect(self.reload)
        self._search_input.textChanged.connect(self._on_search_changed)
        self._search_input.returnPressed.connect(self._find_next)
        self._next_btn.clicked.connect(self._find_next)
        self._prev_btn.clicked.connect(self._find_prev)
        self._case_sensitive_cb.stateChanged.connect(self._on_search_changed)

        self._path: Path | None = None

    def set_file(self, log_file: Path) -> None:
        self._path = log_file
        self._path_label.setText(str(log_file))
        self.reload()

    def reload(self) -> None:
        if not self._path or not self._path.exists():
            name = self._path.name if self._path else "terminal log"
            self._text.setHtml(ansi_to_html(f"No terminal log found ({name}).\n"))
            return

        try:
            data = self._path.read_text(errors="replace")
        except Exception as e:
            self._text.setHtml(ansi_to_html(f"Failed to read log: {e}\n"))
            return

        # Render
        self._original_html = ansi_to_html(data)
        self._text.setHtml(self._original_html)
        # move to top (beginning of log)
        cursor = self._text.textCursor()
        cursor.movePosition(cursor.Start)
        self._text.setTextCursor(cursor)
        self._text.verticalScrollBar().setValue(0)
        # Re-run search if there's a query
        if self._search_input.text():
            self._on_search_changed()

    def _on_search_changed(self) -> None:
        """Called when search text or case sensitivity changes."""
        query = self._search_input.text()
        self._search_positions.clear()
        self._current_match_idx = -1

        if not query:
            self._match_label.setText("")
            # Clear extra selections
            self._text.setExtraSelections([])
            return

        # Find all matches using QTextDocument's find
        doc = self._text.document()
        flags = QtGui.QTextDocument.FindFlags()
        if self._case_sensitive_cb.isChecked():
            flags |= QtGui.QTextDocument.FindCaseSensitively

        cursor = QtGui.QTextCursor(doc)
        while True:
            cursor = doc.find(query, cursor, flags)
            if cursor.isNull():
                break
            # Store start position and selection length
            self._search_positions.append((cursor.selectionStart(), cursor.selectionEnd()))

        if self._search_positions:
            self._current_match_idx = 0
            self._highlight_matches()
        else:
            self._match_label.setText("0 / 0")
            self._text.setExtraSelections([])

    def _find_next(self) -> None:
        """Navigate to the next match."""
        if not self._search_positions:
            return
        self._current_match_idx = (self._current_match_idx + 1) % len(self._search_positions)
        self._highlight_matches()

    def _find_prev(self) -> None:
        """Navigate to the previous match."""
        if not self._search_positions:
            return
        self._current_match_idx = (self._current_match_idx - 1) % len(self._search_positions)
        self._highlight_matches()

    def _highlight_matches(self) -> None:
        """Highlight all matches using extraSelections and scroll to current."""
        if not self._search_positions or self._current_match_idx < 0:
            return

        # Update match label
        self._match_label.setText(f"{self._current_match_idx + 1} / {len(self._search_positions)}")

        # Create highlight formats
        highlight_format = QtGui.QTextCharFormat()
        highlight_format.setBackground(QtGui.QColor("#5a5a00"))  # Yellow-ish for matches
        
        current_format = QtGui.QTextCharFormat()
        current_format.setBackground(QtGui.QColor("#ff8c00"))  # Orange for current match
        current_format.setForeground(QtGui.QColor("#000000"))

        # Build extra selections list
        extra_selections = []
        for idx, (start, end) in enumerate(self._search_positions):
            selection = QtWidgets.QTextEdit.ExtraSelection()
            cursor = self._text.textCursor()
            cursor.setPosition(start)
            cursor.setPosition(end, QtGui.QTextCursor.KeepAnchor)
            selection.cursor = cursor
            
            if idx == self._current_match_idx:
                selection.format = current_format
            else:
                selection.format = highlight_format
            
            extra_selections.append(selection)

        self._text.setExtraSelections(extra_selections)

        # Scroll to current match
        if self._current_match_idx < len(self._search_positions):
            start, end = self._search_positions[self._current_match_idx]
            nav_cursor = self._text.textCursor()
            nav_cursor.setPosition(start)
            self._text.setTextCursor(nav_cursor)
            self._text.ensureCursorVisible()
