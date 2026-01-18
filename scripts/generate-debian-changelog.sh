#!/bin/bash
# Generate Debian changelog from CHANGELOG.md
#
# Usage: generate-debian-changelog.sh <ros_distro> <version> <build_number> [changelog_path]
#
# This script parses CHANGELOG.md and generates a proper Debian changelog
# with the ROS distro-specific package name.

set -euo pipefail

ROS_DISTRO="${1:?Usage: $0 <ros_distro> <version> <build_number> [changelog_path]}"
VERSION="${2:?Usage: $0 <ros_distro> <version> <build_number> [changelog_path]}"
BUILD_NUMBER="${3:-0}"
CHANGELOG_PATH="${4:-CHANGELOG.md}"

# Package name includes ROS distro
PACKAGE_NAME="ros-${ROS_DISTRO}-edgefirst-msgs"

# Maintainer info
MAINTAINER_NAME="${DEBFULLNAME:-Sébastien Taylor}"
MAINTAINER_EMAIL="${DEBEMAIL:-sebastien@au-zone.com}"

# Find the script directory to locate CHANGELOG.md
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Use provided path or default
if [[ "${CHANGELOG_PATH}" != /* ]]; then
    CHANGELOG_PATH="${REPO_ROOT}/${CHANGELOG_PATH}"
fi

if [[ ! -f "${CHANGELOG_PATH}" ]]; then
    echo "Warning: ${CHANGELOG_PATH} not found, generating minimal changelog" >&2
    DATE=$(date -R)
    cat <<EOF
${PACKAGE_NAME} (${VERSION}-${BUILD_NUMBER}) stable; urgency=medium

  * Release ${VERSION}

 -- ${MAINTAINER_NAME} <${MAINTAINER_EMAIL}>  ${DATE}
EOF
    exit 0
fi

# Extract changes for the specified version from CHANGELOG.md
# Returns the content between "## [VERSION]" and the next "## [" header
extract_version_changes() {
    local version="$1"
    local changelog="$2"
    local in_version=0
    local changes=""
    
    while IFS= read -r line; do
        # Check if we hit our version header
        if [[ "$line" =~ ^##[[:space:]]*\[${version}\] ]]; then
            in_version=1
            # Extract date if present (format: ## [1.4.1] - 2025-11-18)
            if [[ "$line" =~ -[[:space:]]*([0-9]{4}-[0-9]{2}-[0-9]{2}) ]]; then
                VERSION_DATE="${BASH_REMATCH[1]}"
            fi
            continue
        fi
        
        # Check if we hit the next version header (end of our section)
        if [[ $in_version -eq 1 && "$line" =~ ^##[[:space:]]*\[ ]]; then
            break
        fi
        
        # Collect changes if we're in the right version
        if [[ $in_version -eq 1 ]]; then
            # Skip empty lines at the start
            if [[ -z "$changes" && -z "${line// }" ]]; then
                continue
            fi
            changes+="${line}"$'\n'
        fi
    done < "$changelog"
    
    echo "$changes"
}

# Convert markdown changes to Debian changelog format
format_debian_changes() {
    local changes="$1"
    local current_section=""
    
    while IFS= read -r line; do
        # Skip empty lines
        [[ -z "${line// }" ]] && continue
        
        # Handle section headers (### Added, ### Changed, etc.)
        if [[ "$line" =~ ^###[[:space:]]*(.*) ]]; then
            current_section="${BASH_REMATCH[1]}"
            continue
        fi
        
        # Handle bullet points
        if [[ "$line" =~ ^[[:space:]]*[-*][[:space:]]+(.*) ]]; then
            local item="${BASH_REMATCH[1]}"
            # Clean up markdown formatting
            item="${item//\*\*/}"  # Remove bold
            item="${item//\`/}"    # Remove code ticks
            
            if [[ -n "$current_section" ]]; then
                echo "  * [${current_section}] ${item}"
            else
                echo "  * ${item}"
            fi
        fi
    done <<< "$changes"
}

# Extract changes for this version
VERSION_DATE=""
CHANGES=$(extract_version_changes "$VERSION" "$CHANGELOG_PATH")

# If no changes found for this version, check Unreleased
if [[ -z "${CHANGES// }" ]]; then
    CHANGES=$(extract_version_changes "Unreleased" "$CHANGELOG_PATH")
fi

# Format the date
if [[ -n "$VERSION_DATE" ]]; then
    # Convert YYYY-MM-DD to RFC 2822 format
    DATE=$(date -R -d "${VERSION_DATE}" 2>/dev/null || date -R -j -f "%Y-%m-%d" "${VERSION_DATE}" 2>/dev/null || date -R)
else
    DATE=$(date -R)
fi

# Generate the debian changelog
echo "${PACKAGE_NAME} (${VERSION}-${BUILD_NUMBER}) stable; urgency=medium"
echo ""

if [[ -n "${CHANGES// }" ]]; then
    format_debian_changes "$CHANGES"
else
    echo "  * Release ${VERSION}"
fi

echo ""
echo " -- ${MAINTAINER_NAME} <${MAINTAINER_EMAIL}>  ${DATE}"
