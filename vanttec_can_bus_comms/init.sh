#!/bin/bash
ip link set can0 type can bitrate 125000 sjw 4 berr-reporting on fd off && sudo ip link set can0 up
