# THIS cmake file generate a library named as the project which includes all the files
# listed for compilation. To change the name of the library, just change the name of the
# project defined in the project(-----) statement.
# 
cmake_minimum_required (VERSION 2.6.4)
set(CMAKE_CPP_CREATE_SHARED_LIBRARY on)
include (FindPkgConfig)

set(SIM_SDK_HOME $ENV{SIMULATOR_SDK_HOME})

message(STATUS ${SIM_SDK_HOME})

project (v_repExtRomeo)

# To link OPENCV
#FIND_PACKAGE( OpenCV REQUIRED )

# Find package qibuild
find_package(qibuild)

#add_subdirectory (v_repExtRomeo) 
set(VREP_ROMEO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

# Define Build Directories
# path where the target will be built (bin and libs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${VREP_ROMEO_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${VREP_ROMEO_BINARY_DIR}/lib/static)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${VREP_ROMEO_BINARY_DIR})

# FROM HERE, BUILDS THE LIBRARY OF THE PLUGIN. 
# The following lines includes the source code of the library.
set(ROMEO_PLUGIN_SRC ../common/v_repLib.cpp src/v_repExtRomeoSensors.cpp src/v_repRomeo.cpp src/v_repExtRomeo.cpp )

#Include simulator-sdk libraries
include_directories(${SIM_SDK_HOME}/include)

#Include vrep libraries
include_directories(${VREP_ROMEO_BINARY_DIR}/include)
include_directories(${VREP_ROMEO_BINARY_DIR}/../include/)

# the following lines adds a new library to the project, using as sources the
# cpp files listed in the previous section. The second line links the
# libraries to the target.
add_library(${PROJECT_NAME} SHARED ${ROMEO_PLUGIN_SRC})
target_link_libraries(${PROJECT_NAME})

# Create the libraries
qi_create_lib(ROMEOAD SHARED SRC ${ROMEO_PLUGIN_SRC})
#qi_use_lib(ROMEOAD ALNAOSIM ALSIMUTILS OPENCV2_CORE OPENCV2_HIGHGUI)
qi_use_lib(ROMEOAD ALNAOSIM ALSIMUTILS)

