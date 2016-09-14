#Specify minimum cmake requirements
cmake_minimum_required(VERSION 2.8.12)

#if(COMMAND cmake_policy)
#    cmake_policy(SET CMP0008 NEW)
#    cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)

project(UAV_mani_epsilon300)

# Pull in standard configuration.
include(CytonBuildDefaults.cmake)

# Pull in rendering requirements
include_directories("${render_inc}"
                    "${PROJECT_SOURCE_DIR}/../include/mavlink/v2.0")

link_directories(${render_lib})
#link_directories(${toolkits}/../lib)#

# Set the contents of our executable
#add_executable(cytonCommandControl
 #  ecCytonCommands.cpp
 #  main.cpp
 #  ecCytonCommands.h
#)

add_executable(APP_epsilon_control
epsilon_control_main.cpp
serial_port.cpp 
autopilot_interface.cpp
ecCytonCommands.cpp
)
# Link appropriate libraries

target_link_libraries(APP_epsilon_control
   pthread
   ecFoundCore
   ecXml
   ecXmlReaderWriter
   ecManipulationDirector
   ecSystemSimulation
   ecSimulation
   ecManipulator
   ecFoundCommon
   ecGeometry
   ecControl
   ecPathPlanning
   ecMatrixUtilities
   ecControlCore
   ecFunction
   ecPlugins
   ecboost_iostreams-mt
   ecboost_filesystem-mt
   ecboost_program_options-mt
   ecboost_system-mt
   ecboost_thread-mt
   ecboost_date_time-mt
   ecRemoteCommand
)

#target_link_libraries(mavlink_control
#                        pthread)
# Link appropriate libraries
if(UNIX)
target_link_libraries(
   APP_epsilon_control
  "${toolkits}/../bin/remoteCommandClientPlugin.ecp"
  "${toolkits}/../bin/imageSensor.ecp"
)

endif(UNIX)

if(WIN32)
target_link_libraries(
   APP_epsilon_control
   "${toolkits}/../lib/remoteCommandClientPlugin.lib"
)
endif(WIN32)

set(CMAKE_BUILD_TYPE Release)
