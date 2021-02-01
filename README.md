## dragon-eye -  F3F automatic base

dragon-eye is an F3F electronic judging system. It's base on technology of computer vision, capture video stream from camera then detect moving objects and keeps tracking them, once one of the tracking target across central vertical line, trigger out a signal.

#### Feature
- Multi moving targets tracking base on technology of background subtraction
- Camera frame rate is fixed to 30 fps. System maximum frame rate is about 50 fps.
- Supports selection 1 of 2 cameras with different angle of view
- Trigger GPIO / UART / UDP when target across central line
- Record video to files with or without tracking result
- built-in RTSP video server
- Video output can be one of the following options HDMI / RTP / HLS / RTSP
- Android APP to start / stop / config / play video

#### Hardware Requirement 
- nVidia Jetson Nano development kit (Prefer version B01)
- AC8265 WIRELESS NIC module (Intel 8265NGW chipset)
- Raspberry pi camera V2 or camera with IMX219 sensor and MIPI-CSI interface
- DC-DC power converter 7~40V to 5V / 5A power
- Panasonic NCR18650PF 3S2P battery pack

#### Software Requirement
- NVIDIA L4T 32.3.1 or later
- OpenCV 4.4.0
- gstreamer-1.x
