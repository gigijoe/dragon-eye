#!/bin/bash

cp -av ./etc/systemd/system/* /etc/systemd/system/
cp -av ./etc/udev/rules.d/* /etc/udev/rules.d/
cp -av ./etc/NetworkManager/NetworkManager.conf /etc/NetworkManager/

cd rtl8188eu
make
sudo make install
cd hostapd-0.8/hostapd/
cp defconfig .config
make
cp -av hostapd /usr/local/bin
cd ../../..

#Copy wifi AP control file of Rtl8188eu 
cp -av ./usr/local/bin/control_ap /usr/local/bin

# Copy camera settings
cp camera_overrides.isp /var/nvidia/nvcam/settings
chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

cp -av build/dragon-eye /usr/local/bin
cp -av ./usr/local/bin/dragon-eye.sh /usr/local/bin

mkdir -p /etc/dragon-eye/
cp -av ./etc/dragon-eye/* /etc/dragon-eye/

systemctl enable rtwap
systemctl enable dragon-eye

mkdir -p /opt/Videos

cp -av ./etc/samba/smb.conf /etc/samba
