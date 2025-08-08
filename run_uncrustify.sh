#!/bin/bash

# ==============================================================================
# This script automatically runs the 'uncrustify' tool on all C++ source
# and header files within the current directory and its subdirectories.
# It uses a configuration file named 'uncrustify.cfg' located in the same
# directory as the script.
# ==============================================================================

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
# Set the name of the uncrustify configuration file.
CONFIG_FILE="uncrustify.cfg"

# Set file extensions to be formatted.
EXTENSIONS=("*.hpp" "*.cpp")

# --- Pre-run checks ---

# Check if the uncrustify executable is available.
if ! command -v uncrustify &> /dev/null; then
    echo "Error: 'uncrustify' is not installed or not in your PATH."
    echo "Please install it, for example:"
    echo "  sudo apt-get install uncrustify"
    exit 1
fi

# Check if the configuration file exists.
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Uncrustify config file '$CONFIG_FILE' not found."
    echo "Please make sure this script is run from the same directory where"
    echo "the config file is located."
    exit 1
fi

# --- Main script logic ---

echo "Starting Uncrustify formatting..."
echo "Using config file: $CONFIG_FILE"

# Find files and run uncrustify on them.
# The `find` command searches for files with the specified extensions in the current
# directory (`.`).
# The `-exec` option runs a command for each file found.
# The `{}` is a placeholder for the filename.
# The `\;` terminates the `-exec` command.
find . -type f \( -name "*.hpp" -o -name "*.cpp" \) -exec uncrustify -c "$CONFIG_FILE" -f {} --replace \;

echo "Uncrustify formatting complete."

