cmake_minimum_required( VERSION 2.8 )
project( DragonEye )
find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )

set( THREADS_PREFER_PTHREAD_FLAG ON )
set( CMAKE_CXX_FLAGS "-O3" )

find_package( Threads REQUIRED )

add_definitions( -std=c++11 )

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB REQUIRED glib-2.0)

include_directories(
  ${GLIB_INCLUDE_DIRS}
)

link_directories(
  ${GLIB_LIBRARY_DIRS}
)

pkg_check_modules(GST REQUIRED 
	gstreamer-1.0
	gstreamer-base-1.0
	gstreamer-rtsp-server-1.0)

include_directories("${GST_INCLUDE_DIRS}")

link_libraries(${GST_LIBRARIES})

set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)

INCLUDE_DIRECTORIES(/usr/local/include/opencv4)
#INCLUDE_DIRECTORIES(/usr/include/glib-2.0)
#INCLUDE_DIRECTORIES(/usr/lib/aarch64-linux-gnu/glib-2.0/include)
#INCLUDE_DIRECTORIES(/usr/include/gstreamer-1.0)

#add_subdirectory( jetsonGPIO )

add_executable( dragon-eye dragon-eye.cpp jetsonGPIO/jetsonGPIO.c )
target_link_libraries( dragon-eye ${OpenCV_LIBS} Threads::Threads ${GST_LIBRARIES} ${CURL_LIBRARY})

add_executable( test-launch test-launch.c )
target_link_libraries( test-launch ${GST_LIBRARIES})

install(TARGETS dragon-eye DESTINATION /usr/local/bin)
