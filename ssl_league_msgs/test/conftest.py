"""Make the cmake/ generator scripts importable without installing them as a package."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'cmake'))
