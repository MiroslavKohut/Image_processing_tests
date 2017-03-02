cmake_minimum_required (VERSION 2.6)
project(kinect_testing)

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( libfreenect REQUIRED )
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)

add_subdirectory (logging)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

include_directories("/usr/include/libusb-1.0/"
		${FREENECT_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}OPENNI_INCLUDE_DIRS
		${PCL_INCLUDE_DIRS}
		)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(capture_data capture_data.cpp)

target_link_libraries(capture_data
		LINK_PUBLIC logging
		${OpenCV_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT}
		${FREENECT_LIBRARIES}
		opencv_imgcodecs
		opencv_core
		opencv_videoio
		opencv_highgui
		/usr/local/lib/libopencv_core.so.3.1
		/usr/local/lib/libopencv_imgproc.so.3.1
		)

add_executable(pcl_testing test_pcl.cpp)

target_link_libraries(pcl_testing
		${OpenCV_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT}
		${FREENECT_LIBRARIES}
		${PCL_COMMON_LIBRARIES}
		${PCL_IO_LIBRARIES}
		${PCL_VISUALIZATION_LIBRARIES}
		opencv_imgcodecs
		opencv_core
		opencv_videoio
		opencv_highgui
		/usr/local/lib/libopencv_core.so.3.1
		/usr/local/lib/libopencv_imgproc.so.3.1
		)

add_executable(sort_data sort_data.cpp)

target_link_libraries(sort_data
		LINK_PUBLIC logging
		${OpenCV_LIBRARIES}
		opencv_imgcodecs
		opencv_core
		opencv_videoio
		opencv_highgui
		/usr/local/lib/libopencv_core.so.3.1
		/usr/local/lib/libopencv_imgproc.so.3.1
		)
