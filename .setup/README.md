# Important commands

## Stop current getty service
sudo systemctl stop custom-getty@ttyXBEE.service
sudo systemctl disable custom-getty@ttyXBEE.service

## For example, stop a running TCU service
sudo systemctl stop serial-getty@ttyTCU0.service
sudo systemctl disable serial-getty@ttyTCU0.service

## Stop all agetty services
sudo pkill agetty

## Reload daemon in case it is needed
sudo systemctl daemon-reload

## Start custom getty service
sudo systemctl enable custom-getty@ttyXBEE.service
sudo systemctl start custom-getty@ttyXBEE.service

## See which serial devices are using getty
ps aux | grep agetty

## Create a custom service (the one we actually use)
FILE NAME: /etc/systemd/system/custom-getty@.services

[Unit]
Description=Custom Serial Getty on %I
ConditionPathExists=/dev/ttyXBEE%I

[Service]
ExecStart=-/sbin/agetty -L -8 230400 %I vt220
Type=idle
Restart=always
RestartSec=0
UtmpIdentifier=%I
TTYPath=/dev/%I
TTYReset=yes
TTYVHangup=yes

[Install]
WantedBy=multi-user.target


## Read status of getty service
systemctl status custom-getty@ttyXBEE.service

## Seek lsof serial port's conflicts
lsof | grep ttyXBEE

## Connect via serial (minicom)
minicom -D /dev/ttyXBEE -b 230400 -c on
