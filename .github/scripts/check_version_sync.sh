#!/bin/bash
# Check that Cargo.toml, Python __init__.py, and ROS2 package.xml versions are synchronized

set -e

# Extract version from Cargo.toml
CARGO_VERSION=$(grep '^version = ' Cargo.toml | head -1 | sed 's/version = "\(.*\)"/\1/')

# Extract version from Python __init__.py
PYTHON_VERSION=$(grep '^__version__ = ' edgefirst/schemas/__init__.py | sed 's/__version__ = "\(.*\)"/\1/')

# Extract version from ROS2 package.xml
PACKAGE_XML_VERSION=$(grep '<version>' edgefirst_msgs/package.xml | sed 's/.*<version>\(.*\)<\/version>.*/\1/')

echo "=================================================="
echo "Version Synchronization Check"
echo "=================================================="
echo ""
echo "Cargo.toml version:         $CARGO_VERSION"
echo "Python __init__.py version: $PYTHON_VERSION"
echo "ROS2 package.xml version:   $PACKAGE_XML_VERSION"
echo ""

if [ "$CARGO_VERSION" = "$PYTHON_VERSION" ] && [ "$CARGO_VERSION" = "$PACKAGE_XML_VERSION" ]; then
    echo "✅ All versions are synchronized!"
    echo ""
    exit 0
else
    echo "❌ VERSION MISMATCH!"
    echo ""
    echo "All versions must match across:"
    echo "  - Cargo.toml"
    echo "  - edgefirst/schemas/__init__.py"
    echo "  - edgefirst_msgs/package.xml"
    echo ""
    echo "To fix, use cargo-release to update all versions:"
    echo "  cargo release version <patch|minor|major|<version>>"
    echo ""
    exit 1
fi
