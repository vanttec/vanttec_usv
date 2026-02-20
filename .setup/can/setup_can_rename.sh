#!/bin/bash
# Installs a systemd .link file that renames the CAN interface from can0 to can_vtec.
# A reboot is required for the rename to take effect.
# Run once per device with: sudo bash setup_can_rename.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LINK_FILE="$SCRIPT_DIR/../files/10-can.link"
DEST="/etc/systemd/network/10-can.link"

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo bash $0"
    exit 1
fi

if [ ! -f "$LINK_FILE" ]; then
    echo "Error: $LINK_FILE not found"
    exit 1
fi

cp "$LINK_FILE" "$DEST"
echo "Installed $DEST"

echo "A reboot is required for the CAN interface to be renamed to can_vtec."
