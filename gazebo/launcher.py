"""Launcher utilities for Gazebo 3D drone visualization."""

from __future__ import annotations

import logging
from pathlib import Path
import shutil
import sys
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

GAZEBO_DIR = Path(__file__).resolve().parent
WORLDS_DIR = GAZEBO_DIR / "worlds"
MODELS_DIR = GAZEBO_DIR / "models"


def detect_gazebo_installation() -> Dict[str, Any]:
    """Inspect environment and return detected Gazebo command and binary path."""
    binaries = ["gz", "ign", "gazebo", "gzserver"]
    for bin_name in binaries:
        path = shutil.which(bin_name)
        if path is not None:
            return {"command": bin_name, "path": path, "detected": True}

    return {"command": None, "path": None, "detected": False}


def get_gazebo_launch_command(world_name: str = "drone_world.sdf") -> list[str]:
    """Generate command line arguments to launch Gazebo with the drone world."""
    world_path = WORLDS_DIR / world_name
    info = detect_gazebo_installation()
    cmd = info["command"]

    if cmd == "gz":
        return ["gz", "sim", "-r", str(world_path)]
    elif cmd == "ign":
        return ["ign", "gazebo", "-r", str(world_path)]
    elif cmd == "gazebo":
        return ["gazebo", "--verbose", str(world_path)]
    else:
        # Default recommendation
        return ["gz", "sim", "-r", str(world_path)]


def print_status_and_instructions() -> None:
    """Print detected environment and manual launch instructions."""
    info = detect_gazebo_installation()
    print("=== Gazebo Visualization Integration Status ===")
    if info["detected"]:
        print(f"Detected Gazebo command: {info['command']} ({info['path']})")
        print(f"Launch command: {' '.join(get_gazebo_launch_command())}")
    else:
        print("Gazebo is NOT detected on the current system PATH.")
        print("\nTo launch Gazebo visualization on a machine with Gazebo/WSL2:")
        print(f"1. Set model path: export GZ_SIM_RESOURCE_PATH={MODELS_DIR}:$GZ_SIM_RESOURCE_PATH")
        print(f"2. Launch world: gz sim -r {WORLDS_DIR / 'drone_world.sdf'}")
        print("3. Run simulator: python main.py (or simulation runner with GazeboBridge)")


if __name__ == "__main__":
    print_status_and_instructions()
