from __future__ import annotations

import html
import re
from dataclasses import dataclass


# Minimal ANSI SGR (Select Graphic Rendition) handling for typical colored logs.
# We convert to HTML fragments suitable for QTextEdit.setHtml().

_SGR_RE = re.compile(r"\x1b\[([0-9;]*)m")


@dataclass
class AnsiStyle:
    fg: str | None = None
    bold: bool = False


_ANSI_FG = {
    30: "#000000",
    31: "#cc0000",
    32: "#4e9a06",
    33: "#c4a000",
    34: "#3465a4",
    35: "#75507b",
    36: "#06989a",
    37: "#d3d7cf",
    90: "#555753",
    91: "#ef2929",
    92: "#8ae234",
    93: "#fce94f",
    94: "#729fcf",
    95: "#ad7fa8",
    96: "#34e2e2",
    97: "#eeeeec",
}


def ansi_to_html(text: str) -> str:
    """Convert an ANSI-colored string to HTML.

    Supports common `\x1b[...m` sequences; unknown codes are ignored.
    """

    style = AnsiStyle()
    out: list[str] = ["<pre style='margin:0; font-family: monospace; white-space: pre-wrap;'>"]

    def open_span(s: AnsiStyle) -> str:
        css = []
        if s.fg:
            css.append(f"color: {s.fg}")
        if s.bold:
            css.append("font-weight: 600")
        if not css:
            return ""
        return f"<span style='{'; '.join(css)}'>"

    def close_span(s: AnsiStyle) -> str:
        return "</span>" if (s.fg or s.bold) else ""

    pos = 0
    span_open = False

    for m in _SGR_RE.finditer(text):
        chunk = text[pos : m.start()]
        if chunk:
            out.append(html.escape(chunk))

        codes_str = m.group(1)
        codes = [int(c) for c in codes_str.split(";") if c] if codes_str else [0]

        # close previous span if any style existed
        if span_open:
            out.append("</span>")
            span_open = False

        # update style
        for code in codes:
            if code == 0:
                style = AnsiStyle()
            elif code == 1:
                style.bold = True
            elif code in _ANSI_FG:
                style.fg = _ANSI_FG[code]
            # ignore others

        span = open_span(style)
        if span:
            out.append(span)
            span_open = True

        pos = m.end()

    tail = text[pos:]
    if tail:
        out.append(html.escape(tail))

    if span_open:
        out.append("</span>")

    out.append("</pre>")
    return "".join(out)
