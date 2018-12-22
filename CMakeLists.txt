cmake_minimum_required(VERSION 2.6)
project(PoseEstimation)

add_definitions(-Dlinux)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(./include)
aux_source_directory(./src SRC_FILES)
add_library(${PROJECT_NAME} ${SRC_FILES})

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found.")
endif()
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(pose samples/pose.cpp)
target_link_libraries(pose ${PROJECT_NAME})

add_executable(vpose samples/vpose.cpp)
target_link_libraries(vpose ${PROJECT_NAME})

add_executable(capMono samples/monoCapture.cpp)
target_link_libraries(capMono ${OpenCV_LIBS})

add_executable(capStereo samples/stereoCapture.cpp)
target_link_libraries(capStereo ${OpenCV_LIBS})
