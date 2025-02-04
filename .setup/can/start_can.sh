#!/bin/bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set down can_vtec
sudo ip link set can_vtec type can bitrate 125000
sudo ip link set up can_vtec

