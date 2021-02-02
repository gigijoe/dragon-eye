## dragon-eye -  F3F automatic base

dragon-eye is an F3F real-time electronic judging system. It's base on technology of computer vision, capture video stream from camera then detect moving objects and keeps tracking them, once one of the tracking target across central vertical line, trigger out a signal. 

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
- Built-in RTSP video server
- Video output can be one of the following option HDMI / RTP / HLS / RTSP (Prefer RTSP)
- Supports Android APP to start / stop / config / play RTSP video stream
- Written in c/c++ for running performance
- Background subtraction runnung by GPU to improve real-time performance

#### Hardware Requirement 
- nVidia Jetson Nano development kit (Prefer version B01)
- AC8265 WIRELESS NIC module (Intel 8265NGW chipset)
- Raspberry pi camera V2 or camera with IMX219 sensor (MIPI-CSI interface)
- DC-DC power converter 7~40V to 5V / 5A power
- Panasonic NCR18650PF 3S2P battery pack (Keeps running up to 4+ hours)

#### Software Requirement
- nVidia Jetpack 4.4 [LT4 32.4.3] or later
- OpenCV 4.4.0
- gstreamer-1.x

#### System Requirement
- Keep camera steady as possible
- Keep central of camera view away from continually moving objects such as grass and trees as possible
- Use camera filter or adjust exposure threshold from Android APP in bright sunlight 

#### Donate

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://paypal.me/stevegigijoe)
