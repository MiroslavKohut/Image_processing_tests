# Create a library called "Hello" which includes the source file "hello.cxx".
# The extension is already found. Any number of sources could be listed here.
find_package( Boost REQUIRED COMPONENTS program_options regex )
include_directories( ${Boost_INCLUDE_DIRS} )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

include_directories("/usr/include/libusb-1.0/"
        ${OpenCV_INCLUDE_DIRS}OPENNI_INCLUDE_DIRS
        ${PCL_INCLUDE_DIRS}
        )

add_library (logging logging.cpp logging.h)

target_link_libraries(logging
        ${OpenCV_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        opencv_imgcodecs
        opencv_core
        opencv_videoio
        opencv_highgui
        /usr/local/lib/libopencv_core.so.3.1
        /usr/local/lib/libopencv_imgproc.so.3.1
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${Boost_REGEX_LIBRARY}
        )

target_include_directories (logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
