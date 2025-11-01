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


def main():
    out = convert_urdf_to_xacro('rover.urdf')
    print(f"Generated XACRO: {out}")


if __name__ == "__main__":
    main()
