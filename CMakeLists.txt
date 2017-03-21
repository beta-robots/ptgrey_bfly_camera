cmake_minimum_required(VERSION 2.8.3)

# Cmake policy
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Project name and the type of project
PROJECT(ptgrey_bfly_camera)
message(STATUS "Executig CMakeLists of project  ${PROJECT_NAME}.")

# DEBUG/RELEASE
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RELEASE") 
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(
    catkin REQUIRED 
    COMPONENTS 
        roscpp 
        rospy 
        dynamic_reconfigure 
        std_msgs
        sensor_msgs 
        image_transport 
        cv_bridge
)

#OPenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#Point Grey's SDK. No Find...cmake file
SET (ptgrey_INCLUDE_DIRS "/usr/include/flycapture")
SET (ptgrey_LIBRARIES "flycapture")
INCLUDE_DIRECTORIES(${ptgrey_INCLUDE_DIRS})

# Set source files 
SET(SRCS_LIB
    src/bfly_camera.cpp
)

# SET(SRCS_NODE
#     src/bfly_camera_node.cpp
#     src/bfly_camera_node_main.cpp
# )
        
# add_service_files(
#   FILES
#   ImageAsService.srv
# )

## Generate added messages and services
# generate_messages(
#       DEPENDENCIES 
#       std_msgs 
# )

#dynamic reconfigure cfg file
# generate_dynamic_reconfigure_options(cfg/bfly_camera_params.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS 
        roscpp 
        rospy 
        dynamic_reconfigure 
        std_msgs
        sensor_msgs
        image_transport 
        cv_bridge        
)

#Build library, linked with Point Grey's SDK
ADD_LIBRARY(${PROJECT_NAME}_LIB SHARED 
            ${SRCS_LIB})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_LIB 
                      ${ptgrey_LIBRARIES}
                      ${OpenCV_LIBS})

#Build ROS node executable
# ADD_EXECUTABLE(${PROJECT_NAME} 
#                ${SRCS_NODE})
# add_dependencies(${PROJECT_NAME} 
#                  ${PROJECT_NAME}_gencpp 
#                  ${PROJECT_NAME}_gencfg)               
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
#                       ${PROJECT_NAME}_LIB
#                       ${catkin_LIBRARIES})

#Build examples
# ADD_SUBDIRECTORY(src/examples)

