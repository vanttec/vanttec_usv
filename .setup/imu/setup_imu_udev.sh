#!/bin/bash
# Creates a udev rule that symlinks the IMU's FTDI USB-serial adapter to /dev/ttyIMU.
# Run once per device with: sudo bash setup_imu_udev.sh

set -e

RULE_FILE="/etc/udev/rules.d/99-imu.rules"
RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="FT0C9STB", SYMLINK+="ttyIMU", MODE="0777"'

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo bash $0"
    exit 1
fi

echo "$RULE" > "$RULE_FILE"
echo "Created $RULE_FILE"

udevadm control --reload-rules
udevadm trigger

echo "Udev rules reloaded. /dev/ttyIMU should now be available:"
ls -la /dev/ttyIMU 2>&1 || echo "Warning: /dev/ttyIMU not found. Make sure the IMU is plugged in."
