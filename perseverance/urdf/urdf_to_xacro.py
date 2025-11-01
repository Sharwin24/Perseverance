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


def convert_urdf_to_xacro(urdf_filename: str) -> Path:
    """Convert a URDF in the same folder as this script to a XACRO file.

    Args:
            urdf_filename: Filename of the URDF (e.g., 'rover.urdf').

    Returns:
            Path to the generated XACRO file.
    """
    script_dir = Path(__file__).resolve().parent
    urdf_path = (script_dir / urdf_filename).resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    xacro_path = urdf_path.with_suffix(urdf_path.suffix + ".xacro")

    text = urdf_path.read_text(encoding="utf-8")

    # 1) Ensure robot tag has xmlns:xacro
    # Replace exactly the tag in the spec if present, otherwise inject the attribute into the first <robot ...> tag
    if '<robot name="perseverance"' in text and 'xmlns:xacro=' not in text:
        # Simple exact replacement first
        text = text.replace(
            '<robot name="perseverance">',
            '<robot name="perseverance" xmlns:xacro="http://ros.org/wiki/xacro">',
        )
        # If it wasn't an exact match (attributes present), inject attribute via regex on first <robot ...>
        if 'xmlns:xacro=' not in text:
            text = re.sub(
                r"<robot\b([^>]*)>",
                r"<robot\1 xmlns:xacro=\"http://ros.org/wiki/xacro\">",
                text,
                count=1,
            )
    elif 'xmlns:xacro=' not in text:
        # Generic injection for other robot names if needed
        text = re.sub(
            r"<robot\b([^>]*)>",
            r"<robot\1 xmlns:xacro=\"http://ros.org/wiki/xacro\">",
            text,
            count=1,
        )

    # 2) Insert xacro:property for mesh_dir right after the opening <robot ...> tag
    if 'xacro:property' not in text and 'xacro: property' not in text:
        prop_line = '\n  <xacro:property name="mesh_dir" value="file://$(find perseverance)/meshes"/>'
        # Insert after the first occurrence of ">" of the robot start tag
        m = re.search(r"<robot\b[^>]*>", text)
        if m:
            insert_pos = m.end()
            text = text[:insert_pos] + prop_line + text[insert_pos:]

    # 3) Replace mesh URI prefixes with ${mesh_dir}
    text = text.replace('package://meshes', '${mesh_dir}')
    text = text.replace('package://perseverance/meshes', '${mesh_dir}')

    # Write out the xacro file
    xacro_path.write_text(text, encoding="utf-8")
    return xacro_path


if __name__ == "__main__":
    run_onshape_to_robot()
    out = convert_urdf_to_xacro('rover.urdf')
    print(f"Generated XACRO: {out}")
