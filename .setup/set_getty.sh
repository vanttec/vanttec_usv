#/bin/bash
sudo systemctl daemon-reload

sudo systemctl enable custom-getty@ttyHOLY.service
sudo systemctl start custom-getty@ttyHOLY.service
