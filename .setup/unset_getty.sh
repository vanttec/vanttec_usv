#/bin/bash
sudo systemctl daemon-reload

## Stop current getty service
sudo systemctl stop custom-getty@ttyUSB0.service
sudo systemctl disable custom-getty@ttyUSB0.service

## For example, stop a running TCU service
sudo systemctl stop serial-getty@ttyTCU0.service
sudo systemctl disable serial-getty@ttyTCU0.service

sudo systemctl stop custom-getty@ttyXBEE.service
sudo systemctl disable custom-getty@ttyXBEE.service

sudo systemctl daemon-reload
