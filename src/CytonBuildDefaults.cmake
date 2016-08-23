# Make sure we can find the Actin distribution
#get_filename_component(sriptDir ${CMAKE_CURRENT_LIST_FILE} PATH)

# to build in "program files"
#set(toolkits "${sriptDir}/../../../toolkits")

# to build in user space
set(toolkits $ENV{EC_TOOLKITS})
if("${toolkits}" STREQUAL "")
   message(FATAL_ERROR "Environment variable 'EC_TOOLKITS' is required to be set.")
endif("${toolkits}" STREQUAL "")

#set(src "${toolkits}/examples/src")
set(external "${toolkits}/../external")
set(render_inc "${external}/render_OSS-20160407-gcc4.8-amd64/OSG-3.4.0/include"
               "${external}/render_OSS-20160407-gcc4.8-amd64/Qt-5.6.0/include"
               "${toolkits}/../include"
               "${external}/boost-20160107-gcc4.8-amd64/boost_1_58_0/include")
set(render_lib "${external}/sensor_OSS-20160404-gcc4.8-amd64/OpenCV-2.4.12.3/lib"
               "${external}/sensor_OSS-20160404-gcc4.8-amd64/tiff-4.0.6/lib"
               "${external}/sensor_OSS-20160404-gcc4.8-amd64/libdc1394-2.1.0/lib"
               "${external}/sensor_OSS-20160404-gcc4.8-amd64/libraw1394-2.0.2/lib"
               "${external}/render_OSS-20160407-gcc4.8-amd64/Qt-5.6.0/lib"
               "${external}/render_OSS-20160407-gcc4.8-amd64/OSG-3.4.0/lib"
               "${toolkits}/../lib"
               "${external}/boost-20160107-gcc4.8-amd64/boost_1_58_0/include/../lib")
set(qt_moc_path "${external}/render_OSS-20160407-gcc4.8-amd64/Qt-5.6.0/bin")

# Set directory to include headers
#include_directories("${toolkits}/../include" "${external}/boost-20160107-gcc4.8-amd64/boost_1_58_0/include")

# Set directory to library files.
#link_directories("${toolkits}/../lib")
#link_directories("${external}/boost-20160107-gcc4.8-amd64/boost_1_58_0/include/../lib")

set(qt_lib Qt5Core Qt5Gui Qt5Widgets)
set(osg_lib osg)

add_definitions(-DEC_BUILD_SHARED_LIBS)
add_definitions(-DEC_HAVE_ACTIN)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(
    "-DACTIN_VERSION_MAJOR=4"
    "-DACTIN_VERSION_MINOR=0"
    "-DACTIN_VERSION_PATCH=20"
    "-DACTIN_VERSION=\"4.0.20\""
)


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")






