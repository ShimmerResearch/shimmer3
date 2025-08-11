#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$SCRIPT_DIR/.."

VERSION_FILE="$BASE_DIR/version.txt"
VERSION_BASE_FILE="$BASE_DIR/version_base.txt"
HEADER_FILE="$BASE_DIR/../Shimmer_Driver/version.h"

DEFAULT_BASE="1.00"
DEFAULT_VERSION="1.00.000"

INCREMENT_TYPE=${1:-patch}

# Ensure version_base.txt exists
if [[ ! -f "$VERSION_BASE_FILE" ]]; then
    echo "$DEFAULT_BASE" > "$VERSION_BASE_FILE"
fi

# Ensure version.txt exists
if [[ ! -f "$VERSION_FILE" ]]; then
    echo "$DEFAULT_VERSION" > "$VERSION_FILE"
fi

# Validate base version format
if ! [[ $(cat "$VERSION_BASE_FILE") =~ ^[0-9]+\.[0-9]+$ ]]; then
    echo "Error: $VERSION_BASE_FILE format invalid (expected MAJOR.MINOR)"
    exit 1
fi

# Validate current version format
if ! [[ $(cat "$VERSION_FILE") =~ ^[0-9]+\.[0-9][0-9]\.[0-9][0-9][0-9]$ ]]; then
    echo "Error: $VERSION_FILE format invalid (expected MAJOR.MINOR.PATCH)"
    exit 1
fi

BASE_VERSION=$(cat "$VERSION_BASE_FILE")
IFS='.' read -r MAJOR_BASE MINOR_BASE <<< "$BASE_VERSION"

CURRENT_VERSION=$(cat "$VERSION_FILE")
IFS='.' read -r MAJOR MINOR PATCH <<< "$CURRENT_VERSION"
unset IFS

# Force decimal interpretation to avoid octal issues
MAJOR=$((10#$MAJOR))
MINOR=$((10#$MINOR))
PATCH=$((10#$PATCH))

case "$INCREMENT_TYPE" in
    major)
        MAJOR=$((MAJOR + 1))
        MINOR=0
        PATCH=0
        # Update version_base.txt for major
        echo "$(printf "%d.%02d" "$MAJOR" "$MINOR")" > "$VERSION_BASE_FILE"
        ;;
    minor)
        MINOR=$((MINOR + 1))
        PATCH=0
        # Update version_base.txt for minor
        echo "$(printf "%d.%02d" "$MAJOR" "$MINOR")" > "$VERSION_BASE_FILE"
        ;;
    patch)
        PATCH=$((PATCH + 1))
        # Do not update version_base.txt for patch
        ;;
    *)
        echo "Unknown increment type: $INCREMENT_TYPE"
        exit 1
        ;;
esac

# Enforce 2-byte major and 1-byte minor/patch limits
if (( MAJOR > 65535 )); then
    echo "Error: Major version exceeds 2-byte limit"
    exit 1
fi
if (( MINOR > 255 )) || (( PATCH > 255 )); then
    echo "Error: Minor/Patch exceeds 1-byte limit"
    exit 1
fi

NEW_VERSION=$(printf "%d.%02d.%03d" "$MAJOR" "$MINOR" "$PATCH")

echo "$NEW_VERSION" > "$VERSION_FILE"

HEADER_DIR=$(dirname "$HEADER_FILE")
if [[ ! -d "$HEADER_DIR" ]]; then
    mkdir -p "$HEADER_DIR"
fi

cat <<EOF > "$HEADER_FILE" || { echo "Failed to write $HEADER_FILE"; exit 1; }
/* Auto-generated version header */
#ifndef VERSION_H
#define VERSION_H

#define FW_VERSION_MAJOR  $MAJOR  //16-bit
#define FW_VERSION_MINOR  $MINOR  //8-bit
#define FW_VERSION_PATCH  $PATCH //8-bit
#define FW_VERSION_STRING "v$NEW_VERSION"

#include <stdint.h>

typedef struct
{
  uint16_t major;
  uint8_t minor;
  uint8_t patch;
} firmware_version_t;

__attribute__((section(".version"), used)) static const firmware_version_t fw_version_struct
    = { .major = FW_VERSION_MAJOR, .minor = FW_VERSION_MINOR, .patch = FW_VERSION_PATCH };

#endif //VERSION_H
EOF

echo "Updated version to $NEW_VERSION"
