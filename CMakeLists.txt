cmake_minimum_required( VERSION 2.8 )
project( DragonEye )
find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )

set( THREADS_PREFER_PTHREAD_FLAG ON )
find_package( Threads REQUIRED )

add_definitions( -std=c++11 )

INCLUDE_DIRECTORIES(/usr/local/include/opencv4)

add_executable( dragon-eye dragon-eye.cpp)
target_link_libraries( dragon-eye ${OpenCV_LIBS} Threads::Threads)

