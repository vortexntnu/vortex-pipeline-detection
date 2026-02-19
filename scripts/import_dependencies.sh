#!/bin/bash

# Import external repository dependencies using vcstool
# This script ensures all required dependencies are present before building

set -e

echo "======================================================================"
echo " Importing External Repository Dependencies"
echo "======================================================================"

# Check if vcstool is installed
if ! command -v vcs &> /dev/null; then
    echo "ERROR: vcstool is not installed"
    echo "Install with: sudo apt-get install python3-vcstool"
    echo "Or: pip install vcstool"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPOS_FILE="$WORKSPACE_DIR/vcstool/dependencies.repos"

# Import repositories
echo "Importing repositories from vcstool/dependencies.repos..."
vcs import --input "$REPOS_FILE" "$WORKSPACE_DIR"

echo "======================================================================"
echo " Dependencies imported successfully!"
echo "======================================================================"
