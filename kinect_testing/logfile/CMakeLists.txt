# Create a library called "Hello" which includes the source file "hello.cxx".
# The extension is already found. Any number of sources could be listed here.
include_directories("/usr/include/libusb-1.0/"
        ${OpenCV_INCLUDE_DIRS}OPENNI_INCLUDE_DIRS
        ${PCL_INCLUDE_DIRS}
        )

add_library (logfile logfile.cpp logfile.h)

target_link_libraries(logfile
        ${OpenCV_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        opencv_imgcodecs
        opencv_core
        opencv_videoio
        opencv_highgui
        /usr/local/lib/libopencv_core.so.3.1
        /usr/local/lib/libopencv_imgproc.so.3.1
        )

target_include_directories (logfile PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
