#!/bin/bash
# Check that every version declaration follows the workspace Cargo.toml version.
#
# - Every crate inherits the workspace version (`version.workspace = true`).
# - Python takes its version from Cargo through maturin (`dynamic = ["version"]`).
# - ROS2 package.xml carries the release part only (REP 149 allows no
#   pre-release), so 4.1.0-rc.1 expects <version>4.1.0</version>.

set -e

# Extract version from the [workspace.package] section of Cargo.toml
CARGO_VERSION=$(sed -n '/^\[workspace.package\]/,/^\[/s/^version = "\(.*\)"/\1/p' Cargo.toml)
RELEASE_VERSION=${CARGO_VERSION%%[-+]*}

# Extract version from ROS2 package.xml
PACKAGE_XML_VERSION=$(grep '<version>' edgefirst_msgs/package.xml | sed 's/.*<version>\(.*\)<\/version>.*/\1/')

echo "=================================================="
echo "Version Synchronization Check"
echo "=================================================="
echo ""
echo "Cargo.toml workspace version:    $CARGO_VERSION"
echo "ROS2 package.xml version:        $PACKAGE_XML_VERSION"
echo ""

FAILED=0

for manifest in crates/*/Cargo.toml; do
    if ! grep -q '^version.workspace = true' "$manifest"; then
        echo "❌ $manifest does not inherit the workspace version (version.workspace = true)"
        FAILED=1
    fi
done

for pyproject in pyproject.toml crates/python/pyproject.toml; do
    if grep -q '^version = ' "$pyproject" || ! grep -q '^dynamic = \["version"\]' "$pyproject"; then
        echo "❌ $pyproject must take its version from Cargo (dynamic = [\"version\"])"
        FAILED=1
    fi
done

if [ "$PACKAGE_XML_VERSION" != "$RELEASE_VERSION" ]; then
    echo "❌ edgefirst_msgs/package.xml is $PACKAGE_XML_VERSION, expected $RELEASE_VERSION"
    FAILED=1
fi

if [ "$FAILED" -eq 0 ]; then
    echo "✅ All versions are synchronized!"
    echo ""
    exit 0
else
    echo ""
    echo "To fix, use cargo-release to update all versions:"
    echo "  cargo release version <patch|minor|major|<version>>"
    echo ""
    exit 1
fi
