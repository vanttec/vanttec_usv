#/bin/bash
sudo systemctl daemon-reload

sudo systemctl enable custom-getty@ttyXBEE.service
sudo systemctl start custom-getty@ttyXBEE.service
