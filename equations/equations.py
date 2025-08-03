#!/usr/bin/env python3
"""
Script to replace transparent backgrounds in SVG files with white backgrounds.
Create SVG images for equations at https://latex.codecogs.com/editor.html
"""

import os
import re
from xml.etree import ElementTree as ET

BG_COLOR = "#F0F6FC"


def add_white_background_to_svg(svg_file_path):
    """
    Add a white background to an SVG file by inserting a white rectangle
    that covers the entire viewBox.

    Args:
        svg_file_path (str): Path to the SVG file to modify
    """
    try:
        # Register namespaces to preserve them
        ET.register_namespace('', 'http://www.w3.org/2000/svg')
        ET.register_namespace('xlink', 'http://www.w3.org/1999/xlink')

        # Parse the SVG file
        tree = ET.parse(svg_file_path)
        root = tree.getroot()

        # Get the viewBox dimensions
        viewbox = root.get('viewBox')
        if not viewbox:
            print(f"Warning: No viewBox found in {svg_file_path}")
            return False

        # Parse viewBox: "x y width height"
        viewbox_parts = viewbox.split()
        if len(viewbox_parts) != 4:
            print(f"Warning: Invalid viewBox format in {svg_file_path}")
            return False

        x, y, width, height = map(float, viewbox_parts)

        # Add more padding around the image
        padding = 50  # Increased padding from 20 to 50
        padded_x = x - padding
        padded_y = y - padding
        padded_width = width + (2 * padding)
        padded_height = height + (2 * padding)

        # Create a white background rectangle with rounded corners
        background_rect = ET.Element('rect')
        background_rect.set('x', str(padded_x))
        background_rect.set('y', str(padded_y))
        background_rect.set('width', str(padded_width))
        background_rect.set('height', str(padded_height))
        background_rect.set('fill', BG_COLOR)
        background_rect.set('rx', '8')  # 8px rounded corners
        background_rect.set('ry', '8')  # 8px rounded corners

        # Insert the background rectangle as the first child
        # This ensures it appears behind all other elements
        root.insert(0, background_rect)

        # Write the modified SVG back to file
        tree.write(svg_file_path, encoding='utf-8', xml_declaration=True)

        print(f"✓ Added white background to {svg_file_path}")
        return True

    except Exception as e:
        print(f"✗ Error processing {svg_file_path}: {e}")
        return False


def process_svg_files_in_directory(directory_path):
    """
    Process all SVG files in a directory to add white backgrounds.

    Args:
        directory_path (str): Path to the directory containing SVG files
    """
    svg_files = []

    # Find all SVG files in the directory
    for file in os.listdir(directory_path):
        if file.lower().endswith('.svg'):
            svg_files.append(os.path.join(directory_path, file))

    if not svg_files:
        print(f"No SVG files found in {directory_path}")
        return

    print(f"Found {len(svg_files)} SVG file(s) to process:")
    for svg_file in svg_files:
        print(f"  - {os.path.basename(svg_file)}")

    print("\nProcessing files...")

    success_count = 0
    for svg_file in svg_files:
        if add_white_background_to_svg(svg_file):
            success_count += 1

    print(
        f"\nCompleted: {success_count}/{len(svg_files)} files processed successfully.")


if __name__ == "__main__":
    # Process SVG files in the current working directory
    current_dir = os.getcwd()

    print(f"Processing SVG files in: {current_dir}")
    process_svg_files_in_directory(current_dir)
