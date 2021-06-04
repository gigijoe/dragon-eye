#!/bin/bash

sudo cp -av ./etc/systemd/system/* /etc/systemd/system/
sudo cp -av ./etc/udev/rules.d/* /etc/udev/rules.d/
sudo cp -av ./etc/NetworkManager/NetworkManager.conf /etc/NetworkManager/

cd rtl8188eu
make
sudo make install
cd hostapd-0.8/hostapd/
cp defconfig .config
make
sudo cp -av hostapd /usr/local/bin
cd ../../..

#Copy wifi AP control file of Rtl8188eu 
sudo cp -av ./usr/local/bin/control_ap /usr/local/bin

# Copy camera settings
sudo cp camera_overrides.isp /var/nvidia/nvcam/settings
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

sudo cp -av build/dragon-eye /usr/local/bin
sudo cp -av ./usr/local/bin/dragon-eye.sh /usr/local/bin

sudo mkdir -p /etc/dragon-eye/
sudo cp -av ./etc/dragon-eye/* /etc/dragon-eye/

sudo systemctl enable rtwap
sudo systemctl enable dragon-eye

sudo mkdir -p /opt/Videos
sudo chmod 775 /opt/Videos

sudo cp -av ./etc/samba/smb.conf /etc/samba
sudo smbpasswd -a gigijoe
