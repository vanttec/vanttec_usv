#!/bin/bash

# Remove previous LiDAR route if it exists
sudo ip route del 192.168.1.201 dev enP8p1s0 2>/dev/null || true

# Flush any existing IP on the interface
sudo ip addr flush dev enP8p1s0

# Add IP with /32 netmask (no subnet route created!)
sudo ip addr add 192.168.1.200/32 dev enP8p1s0

# Bring interface up
sudo ip link set enP8p1s0 up

# Add only the specific route to the LiDAR
sudo ip route add 192.168.1.201 dev enP8p1s0

# IPV4 should be set manually to 192.168.1.201 / 24
