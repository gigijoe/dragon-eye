#!/bin/sh
sudo apt-get update
sudo apt-get install cmake git
sudo apt-get install python-pip
sudo apt-get install python3-pip python3-pil

sudo apt-get install gstreamer1.0-tools gstreamer1.0-alsa \
gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
gstreamer1.0-libav

sudo apt-get install libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev \
libgstreamer-plugins-good1.0-dev \
libgstreamer-plugins-bad1.0-dev \
libgstrtspserver-1.0-dev

sudo apt-get install libgtk2.0-dev

sudo apt-get install cmake

sudo apt-get install libcurl4-openssl-dev

sudo -H pip3 install jetson-stats

sudo apt-get install samba -y

wget https://github.com/opencv/opencv/archive/4.4.0.zip
wget https://github.com/opencv/opencv_contrib/archive/4.4.0.zip

unzip opencv-4.4.0.zip
unzip opencv_contrib-4.4.0.zip

cd opencv-4.4.0
mkdir build
cd build

cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.4.0/modules -DWITH_CUDA=ON -DCUDA_FAST_MATH=1 -DBUILD_EXAMPLES=OFF -DWITH_GSTREAMER=ON -DWITH_V4L=ON -DWITH_LIBV4L=OFF -D BUILD_opencv_python2=ON -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_opencv_python3=ON -DPYTHON3_INCLUDE_DIR2=/usr/include/python3.6m -DPYTHON3_NUMPY_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -DENABLE_CXX11=ON -D CMAKE_C_COMPILER=/usr/bin/gcc-7 ..

make -j $(($(nproc) + 1))
sudo make install

cd ../..

sudo systemctl disable gdm3
sudo systemctl set-default multi-user.target
