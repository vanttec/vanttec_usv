# usv_comms

To setup the xbees port names correctly, follow these steps:
1. Create the following file: /etc/udev/rules.d/99-usb-serial.rules
sudo vim /etc/udev/rules.d/99-usb-serial.rules

2. Run the following to edit the file:
sudo nano /etc/udev/rules.d/99-usb-serial.rules

3. Write the following in the file:
SUBSYSTEM=="usb", ATTRS{idProduct}=="6015", ATTRS{idVendor}=="0403", ATTRS{serial}=="D309S1FR", SYMLINK="xbee_boat", NAME="xbee_boat"
SUBSYSTEM=="usb", ATTRS{idProduct}=="6015", ATTRS{idVendor}=="0403", ATTRS{serial}=="D309R0S4", SYMLINK="xbee_station", NAME="xbee_station"

4. Unplug the xbee device(s), and run: 
sudo /etc/init.d/udev restart

NOTE: To check the properties of other usb devices run:
udevadm info -a /dev/ttyUSB0
or
lsusb
