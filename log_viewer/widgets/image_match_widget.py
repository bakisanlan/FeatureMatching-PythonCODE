from __future__ import annotations

from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets


class ImageMatchWidget(QtWidgets.QWidget):
    """Browse measurement update images under logs/<run>/ImageMatch."""

    def __init__(self, parent: QtWidgets.QWidget | None = None):
        super().__init__(parent)

        self._images: list[Path] = []
        self._idx: int = -1

        self._title = QtWidgets.QLabel("ImageMatch")
        self._title.setObjectName("panelTitle")

        self._path_label = QtWidgets.QLabel("-")
        self._path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        self._image_label = QtWidgets.QLabel()
        self._image_label.setAlignment(QtCore.Qt.AlignCenter)
        self._image_label.setWordWrap(True)
        self._image_label.setTextFormat(QtCore.Qt.PlainText)
        self._image_label.setMinimumHeight(300)
        self._image_label.setStyleSheet("background: #262a31; border: 1px solid #404651;")

        self._prev_btn = QtWidgets.QToolButton()
        self._prev_btn.setText("←")
        self._prev_btn.setToolTip("Previous image (Left arrow)")
        self._next_btn = QtWidgets.QToolButton()
        self._next_btn.setText("→")
        self._next_btn.setToolTip("Next image (Right arrow)")

        self._index_label = QtWidgets.QLabel("0 / 0")

        nav = QtWidgets.QHBoxLayout()
        nav.addWidget(self._prev_btn)
        nav.addWidget(self._next_btn)
        nav.addStretch(1)
        nav.addWidget(self._index_label)

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(self._title)
        outer.addWidget(self._path_label)
        outer.addLayout(nav)
        outer.addWidget(self._image_label, 1)

        self._prev_btn.clicked.connect(self.prev_image)
        self._next_btn.clicked.connect(self.next_image)

        # keyboard shortcuts
        QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Left), self, self.prev_image)
        QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Right), self, self.next_image)

    def set_directory(self, image_dir: Path) -> None:
        self._path_label.setText(str(image_dir))

        if not image_dir.exists() or not image_dir.is_dir():
            self._images = []
            self._idx = -1
            self._update_view()
            return

        # measurement_update_<timestamp>.png
        imgs = sorted(image_dir.glob("measurement_update_*.png"))
        if not imgs:
            # fallback: any png/jpg
            imgs = sorted(list(image_dir.glob("*.png")) + list(image_dir.glob("*.jpg")) + list(image_dir.glob("*.jpeg")))

        self._images = imgs
        self._idx = 0 if self._images else -1
        self._update_view()

    def prev_image(self) -> None:
        if not self._images:
            return
        self._idx = max(0, self._idx - 1)
        self._update_view()

    def next_image(self) -> None:
        if not self._images:
            return
        self._idx = min(len(self._images) - 1, self._idx + 1)
        self._update_view()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._update_pixmap()

    def _update_view(self) -> None:
        if not self._images or self._idx < 0:
            self._index_label.setText("0 / 0")
            # Clear any previous pixmap first so the message is actually visible.
            self._image_label.setPixmap(QtGui.QPixmap())
            self._image_label.setText("No ImageMatch images found\n\n(Expected: measurement_update_*.png)")
            self._prev_btn.setEnabled(False)
            self._next_btn.setEnabled(False)
            return

        self._prev_btn.setEnabled(self._idx > 0)
        self._next_btn.setEnabled(self._idx < len(self._images) - 1)
        self._index_label.setText(f"{self._idx + 1} / {len(self._images)}")

        self._update_pixmap()

    def _update_pixmap(self) -> None:
        if not self._images or self._idx < 0:
            return

        img_path = self._images[self._idx]
        pm = QtGui.QPixmap(str(img_path))
        if pm.isNull():
            self._image_label.setText(f"Failed to load: {img_path.name}")
            self._image_label.setPixmap(QtGui.QPixmap())
            return

        w = max(10, self._image_label.width() - 10)
        h = max(10, self._image_label.height() - 10)
        scaled = pm.scaled(w, h, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self._image_label.setPixmap(scaled)
        self._image_label.setText("")
