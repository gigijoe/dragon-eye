## dragon-eye -  F3F real-time electronic judging system with Jetson Nano

dragon-eye is an F3F real-time electronic judging system with Jetson Nano. It's base on technology of computer vision, capture video stream from camera then detect moving objects and keeps tracking them, once one of the tracking target across central vertical line, trigger out a signal. 

#### [F3F - SLOPE SOARING GLIDERS](https://www.fai.org/page/f3-radio-control-soaring)

As the name depicts, this event is flown in front of a slope, taking advantage of the updrafts created by the wind. In the single Speed task the pilot tries to fly its model over a 1000m course (composed of 10 X 100 meter legs) in the shortest possible time.

At least four rounds should be completed in order to determine the winner.

#### Feature
- Multi moving targets tracking base on technology of background subtraction (MOG2)
- Camera resolution is 720p 30fps on Jetson nano / 1080p 30 fps on Jetson Xavier NX
- Supports selection 1 of 2 cameras with different angle of view
- Trigger out GPIO (Relay) / UART / UDP when target across central line
- Record video files to SD card with or without tracking result
- Built-in wifi AP for connectivity
- Built-in RTSP video server (H.265 codec)
- Video output can be one of the following option HDMI / RTP / HLS / RTSP (Prefer RTSP)
- Android APP to start / stop / config / play RTSP video stream
- Written in c/c++ for running performance
- Background subtraction runnung by GPU to improve real-time performance
- Camera settings for different scenes such as dim light or over exposure
- Adjustable MOG2 threshold to reduce nosie or improve object detection 
- Supports Jetson Nano 2GB Developer Kit for low cost solution
- Access video files through samba (Windows Network Neighborhood)

#### Hardware Standard Requirement 
- nVidia Jetson Nano developer kit (Prefer version B01)
- Cooling fan for Jetson Nano
- AC8265 WIRELESS NIC module (Intel 8265NGW chipset)
- ( Option ) EDIMAX EW-7811Un V2 N150 USB wifi dongle
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 77 degree 
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 160 degree 
- UV lens protector (37mm) x 2
- Lens mount adapter (37mm to 34mm) x 2
- DC-DC power converter 7~40V to 5V / 5A power
- Panasonic NCR18650PF 3S2P battery pack (Keeps running up to 8+ hours)
- 64GB up SDHC card

#### Hardware Cost-Down Requirement
- nVidia Jetson Nano 2GB developer kit
- Cooling fan for Jetson Nano
- EDIMAX EW-7811Un V2 N150 USB wifi dongle
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 77 degree / FoV 160 degree
- UV lens protector (37mm)
- Lens mount adapter (37mm to 34mm)
- DC-DC power converter 7~40V to 5V / 5A power
- Panasonic NCR18650PF 3S2P battery pack (Keeps running up to 8+ hours)
- 64GB up SDHC card

#### Hardware High-end Requirement
- nVidia Jetson Xavier NX developer kit
- ( Option ) EDIMAX EW-7811Un V2 N150 USB wifi dongle
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 77 degree
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 160 degree
- UV lens protector (37mm) x 2
- Lens mount adapter (37mm to 34mm) x 2
- Panasonic NCR18650PF 4S2P battery pack
- 64GB up SDHC card

#### Software Requirement
- nVidia Jetpack 4.5.1 [LT4 32.5.1]
- OpenCV 4.4.0
- gstreamer-1.x

#### System Requirement
- Keep camera steady as possible
- Keep central of camera view away from continually moving objects such as grass and trees as possible
- Use camera filter or adjust exposure threshold from Android APP in bright sunlight 

#### Android APP
[dragon-eye-rc](https://github.com/gigijoe/dragon-eye-rc)
- Remote control dragon-eye from Android phone
- Connect dragon-eye through wifi
- Start / Stop dragon-eye
- System config / Camera config
- Play video from RTSP server of dragon-eye
- Play sound with trigger
- F3F Timer 
- Upgrade dragon-eye firmware

[Demo Video](https://youtu.be/_YH6i1c2meU)

#### Simulator
[dragon-eye simulator](https://github.com/gigijoe/dragon-eye-simulator)
- An program running on Ubuntu desktop for development of dragon-eye
- It takes video file recorded by dragon-eye and output result on screen or to file

#### I/O pin

Jetson nano
```
RED LED - 		GPIO16
GREEN LED - 	GPIO17
Button - 		GPIO18 
BLUE LED - 		GPIO50
RELAY -			GPIO51

UART2_TX - 		TTL TX
UART2_RX - 		TTL RX
```

Jetson Xavier NX
```
RED LED - 		GPIO493
GREEN LED - 	GPIO492
Button - 		GPIO491
BLUE LED - 		GPIO428
RELAY -			GPIO429

UART1_TX - 		TTL TX
UART1_RX - 		TTL RX
```

#### Software Prepare

[Jetson Nano Initial Setup](https://stevegigijoe.blogspot.com/2019/05/jetson-nano-initial-setup.html)

[Jetson Nano Camera Setup](https://stevegigijoe.blogspot.com/2019/05/jetson-naon-camera-support.html)

[Jetson Nano GPIO Setup](https://stevegigijoe.blogspot.com/2019/06/jetson-nano-gpio-support.html)

[Jetson Nano Wifi Hotspot](https://stevegigijoe.blogspot.com/2020/07/jetson-nano-wifi-hotspot.html)

Disable GUI

```
sudo systemctl disable gdm3
sudo systemctl set-default multi-user.target
```

#### Build & Run

```
git clone --recursive https://github.com/gigijoe/dragon-eye.git

cd dragon-eye
mkdir build
cd build
cmake ../
make
sudo make install
sudo ./install.sh
sudo reboot
```

#### TODO
- 3D print camera mount 

#### Notice of Setup


#### MOG2 Threshold

The ‘MOG2 threshold’ setting is the adjustment of background subtraction, the lower the value the more sensitive the camera, but the more noise in the image. The suggested threshold value is between 8 to 32.

#### New Target Restriction

‘New Target Restriction’ is a rectangle region (360 x 180pixals) in the bottom central area of camera view in which new targets will not be detected. It is helpful to prevent false triggers when there‘s grass in bottom of camera view.

#### Fake Target Detection

‘Fake Target Detection‘ records all new targets in the previous 3 seconds, if there are targets that overlap too many times these targets will be treated as a fake target and will not trigger the system.

#### Fly Bug Detection

‘Fly Bug Detection‘ is to prevent false triggers caused by bugs flying through the field of view. Bugs are usually small and fast, so targets that are small in size and at high speed will not cause a trigger when they cross the central line.

#### Donate

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://paypal.me/stevegigijoe)
