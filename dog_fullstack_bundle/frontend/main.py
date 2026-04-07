import sys
from pathlib import Path

# Make the bundled backend/python importable without PYTHONPATH.
BACKEND_PY = Path(__file__).resolve().parents[1] / "backend" / "python"
if str(BACKEND_PY) not in sys.path:
    sys.path.insert(0, str(BACKEND_PY))

from PySide6.QtWidgets import QApplication
from app.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
