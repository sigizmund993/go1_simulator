#!/usr/bin/env python3
"""Referee controller entry point for the simulator: the judge from TrueTechArena, unchanged.
Runs in its own supervisor process, separate from the robot's controller."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from referee_core import Referee   # noqa: E402

Referee().run()
