"""Make the python packages under linear_mpc_controller/ importable when running pytest
from the package root (no pip install required).

Run tests from the linear_mpc_controller/ directory:
    python -m pytest mpc_core trajectory_tools benchmark_tools -q
"""
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
