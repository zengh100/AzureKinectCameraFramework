cmake_minimum_required(VERSION 2.8)
# make sure the follow installed
# opencv
# k4a (azure kinect SDK)

set(k4a_DIR "/usr/lib/x86_64-linux-gnu/cmake/k4a")
find_package( k4a REQUIRED )

set(k4arecord_DIR "/usr/lib/x86_64-linux-gnu/cmake/k4arecord")
find_package( k4arecord REQUIRED )

find_package( OpenCV REQUIRED )
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

#find_package(Boost COMPONENTS system filesystem REQUIRED)
#message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")
#message(STATUS "Boost_LIBS = ${Boost_LIBS}")

## Use the variable PROJECT_NAME for changing the target name
set( PROJECT_NAME "AzureKinectK4Wrapper" )

## Set our project name
project(${PROJECT_NAME})
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
#    ${Boost_INCLUDE_DIRS}
)
## Use all the *.cpp files we found under this folder for the project
#FILE(GLOB SRCS "*.cpp" "*.c")

## Define the executable
#add_executable(${PROJECT_NAME} ${SRCS})
add_executable( ${PROJECT_NAME} 
	src/main_demo.cpp 
	src/time2string.cpp 
	src/AzureKinectCamera.cpp
	)
target_link_libraries( 
      ${PROJECT_NAME}  
      ${OpenCV_LIBS}
      k4a::k4a
      k4a::k4arecord
      #Boost::system
      #Boost::filesystem
      )

# For shared library setting, you may also link cuda if it is available.
 find_package(CUDA)
 if(CUDA_FOUND)
   target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})
 endif()
