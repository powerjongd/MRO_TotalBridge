"""Console entry point for Unified Bridge."""
from __future__ import annotations

import sys

from PySide6 import QtWidgets

from unified_bridge.application import run


def main(argv: list[str] | None = None) -> int:
    """Run the Qt application and return the exit code."""

    try:
        exit_code = run(argv)
    finally:
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.deleteLater()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
