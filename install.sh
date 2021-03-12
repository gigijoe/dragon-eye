#!/bin/bash

cp -av ./etc/systemd/system/* /etc/systemd/system/
cp -av ./etc/udev/rules.d/* /etc/udev/rules.d/

#Copy wifi AP files of Rtl8188eu 
cp -av ./usr/local/bin/* /usr/local/bin

# Copy camera settings
cp camera_overrides.isp /var/nvidia/nvcam/settings
chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

cp -av build/dragon-eye /usr/local/bin

mkdir -p /etc/dragon-eye/
cp -av ./etc/dragon-eye/* /etc/dragon-eye/

systemctl enable rtwap
systemctl enable dragon-eye

mkdir -p ${HOME}/Videos

