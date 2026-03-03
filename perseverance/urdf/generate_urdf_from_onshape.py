"""
Convert onshape-to-robot generated URDF to XACRO format.

Procedure implemented (from the inline spec):
- Create a xacro file with same name as URDF file: "<filename>.urdf.xacro"
- Copy the entire contents from the URDF file to the xacro file
- Replace the <robot name="perseverance"> tag with
  '<robot name="perseverance" xmlns:xacro="http://ros.org/wiki/xacro">'
- Add this tag inside the robot tag:
  '<xacro:property name="mesh_dir" value="file://$(find perseverance)/meshes"/>'
- Replace 'package://meshes' with '${mesh_dir}'

Notes:
- For robustness, we also replace 'package://perseverance/meshes' with '${mesh_dir}'.
- If the xmlns:xacro attribute or xacro:property already exist, we avoid duplicating them.
"""

from __future__ import annotations

import os
import re
from pathlib import Path
import subprocess
import shutil


def run_onshape_to_robot(target_dir: str | Path | None = None) -> None:
    """Run onshape-to-robot to (re)generate URDF meshes for the package.

    Behavior:
    - By default, runs inside the perseverance package directory and uses
      `onshape-to-robot .` (matches manual usage you ran earlier).
    - If a local virtual environment exists at repo_root/.venv, we will prefer
      its onshape-to-robot console script; otherwise we use the one on PATH.

    Args:
      target_dir: Directory to run the tool in. Defaults to the package dir.
    """
    script_dir = Path(__file__).resolve().parent  # .../perseverance/urdf
    # .../perseverance
    package_dir = Path(target_dir) if target_dir else script_dir.parent
    if not package_dir.exists():
        raise FileNotFoundError(
            f"Target directory does not exist: {package_dir}")

    repo_root = package_dir.parent  # repo root
    venv_bin = repo_root / ".venv" / "bin"
    venv_tool = venv_bin / "onshape-to-robot"

    # Prefer venv console script if it exists; otherwise fall back to PATH
    if venv_tool.exists():
        cmd_path = str(venv_tool)
    else:
        which = shutil.which("onshape-to-robot")
        if not which:
            raise RuntimeError(
                "onshape-to-robot not found. Install it (preferably in .venv) or add it to PATH.")
        cmd_path = which

    print(f"Running: {cmd_path} . (cwd={package_dir})")
    try:
        subprocess.run([cmd_path, "."], cwd=str(package_dir), check=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(
            f"onshape-to-robot failed with exit code {e.returncode}") from e
    print("onshape-to-robot conversion completed")


def clean_urdf(urdf_filename: str):
    """Clean URDF file generated from onshape-to-robot

    Args:
        urdf_filename: Filename of the URDF (e.g., 'rover.urdf').
    """
    script_dir = Path(__file__).resolve().parent
    urdf_path = (script_dir / urdf_filename).resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    # Read the URDF file content
    text = urdf_path.read_text(encoding="utf-8")

    # Replace onshape-to-robot's package URI with the correct ROS package path
    text = text.replace('package://urdf/', '')
    # Write to the original URDF file (overwriting it)
    urdf_path.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    run_onshape_to_robot()
    clean_urdf('rover.urdf')
    print(f"Cleaned URDF: rover.urdf")
