## dragon-eye -  F3F real-time electronic judging system with Jetson Nano

dragon-eye is an F3F real-time electronic judging system with Jetson Nano. It's base on technology of computer vision, capture video stream from camera then detect moving objects and keeps tracking them, once one of the tracking target across central vertical line, trigger out a signal. 

#### [F3F - SLOPE SOARING GLIDERS](https://www.fai.org/page/f3-radio-control-soaring)

As the name depicts, this event is flown in front of a slope, taking advantage of the updrafts created by the wind. In the single Speed task the pilot tries to fly its model over a 1000m course (composed of 10 X 100 meter legs) in the shortest possible time.

At least four rounds should be completed in order to determine the winner.

#### Feature
- Multi moving targets tracking base on technology of background subtraction (MOG2)
- Camera resolution is 720p and frame rate is limited to 30 fps
- Supports selection 1 of 2 cameras with different angle of view
- Trigger out GPIO / UART / UDP when target across central line
- Record video files to SD card with or without tracking result
- Built-in wifi AP for connectivity
- Built-in RTSP video server (H.265 codec)
- Video output can be one of the following option HDMI / RTP / HLS / RTSP (Prefer RTSP)
- Android APP to start / stop / config / play RTSP video stream
- Written in c/c++ for running performance
- Background subtraction runnung by GPU to improve real-time performance
- Camera settings for different scenes such as dim light or over exposure
- Adjustable MOG2 threshold to reduce nosie or improve object detection 

#### Hardware Standard Requirement 
- nVidia Jetson Nano developer kit (Prefer version B01)
- Cooling fan for Jetson Nano
- ( Option ) AC8265 WIRELESS NIC module (Intel 8265NGW chipset)
- EDIMAX EW-7811Un V2 N150 USB wifi dongle
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 77 degree 
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 160 degree 
- UV lens protector (37mm) x 2
- Lens mount adapter (37mm to 34mm) x 2
- DC-DC power converter 7~40V to 5V / 5A power
- Panasonic NCR18650PF 3S2P battery pack (Keeps running up to 7+ hours)

#### Hardware Cost-Down Requirement
- nVidia Jetson Nano 2GB developer kit
- Cooling fan for Jetson Nano
- EDIMAX EW-7811Un V2 N150 USB wifi dongle
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface) FoV 77 degree / FoV 160 degree
- UV lens protector (37mm)
- Lens mount adapter (37mm to 34mm)
- DC-DC power converter 7~40V to 5V / 5A power
- Panasonic NCR18650PF 3S2P battery pack (Keeps running up to 7+ hours)

#### Software Requirement
- nVidia Jetpack 4.4 [LT4 32.4.3] or later
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

#### Simulator
[dragon-eye simulator](https://github.com/gigijoe/dragon-eye-simulator)
- An program running on Ubuntu desktop for development of dragon-eye
- It takes video file recorded by dragon-eye and output result on screen or to file

#### I/O pin

```
RED LED - 		GPIO16
GREEN LED - 	GPIO17
Button - 		GPIO18 
BLUE LED - 		GPIO50
RELAY -			GPIO51

UART2_TX - 		TTL TX
UART2_RX - 		TTL RX
```

#### Software Prepare

[Jetson Nano Initial Setup](https://stevegigijoe.blogspot.com/2019/05/jetson-nano-initial-setup.html)

[Jetson Nano Camera Setup](https://stevegigijoe.blogspot.com/2019/05/jetson-naon-camera-support.html)

[Jetson Nano GPIO Setup](https://stevegigijoe.blogspot.com/2019/06/jetson-nano-gpio-support.html)

[Jetson Nano Wifi Hotspot](https://stevegigijoe.blogspot.com/2020/07/jetson-nano-wifi-hotspot.html)

Disable GUI

```
sudo systemctl set-default multi-user.target
```

#### Build & Run

```
cd dragon-eye
mkdir build
cd build
cmake ../
make
sudo ./dragon-eye
```

Start / Stop dragon-eye object tracking from Android APP

[Jetson Nano run program on startup](https://stevegigijoe.blogspot.com/2019/07/jetson-nano-run-program-on-startup.html)

#### TODO
- Testing for dual camera 
- Testing with Jetson Nano 2GB Developer Kit for low cost solution
- 3D print camera mount 

#### Notice of Setup

#### MOG2 Threshold

#### Fake Target Detection

#### Bug Trigger

#### Donate

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://paypal.me/stevegigijoe)
