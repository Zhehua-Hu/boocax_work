SET(PROJECT_NAME Sle_monocularSLAM)  

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#set(CMAKE_VERBOSE_MAKEFILE ON)


set(BUILD_TYPE Release)
#set(BUILD_TYPE RelWithDebInfo)
MESSAGE("BUild type:" ${BUILD_TYPE})

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# required libraries
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message("Find_Eigen3")
else()
  message(FATAL_ERROR "Not Find Eigen3")
endif()

# optional libraries
#find_package(Pangolin QUIET)
find_package(OpenCV 3.0 QUIET)

if(OpenCV_FOUND)
  message("Find_OpenCV")
else()
  message(FATAL_ERROR "Not Find OpenCV")
endif()


# Sources files
set(SLEmonocular_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/map.cpp
  ${PROJECT_SOURCE_DIR}/src/frame.cpp
  ${PROJECT_SOURCE_DIR}/src/config.cpp 
  ${PROJECT_SOURCE_DIR}/src/visual_odometry.cpp
  ${PROJECT_SOURCE_DIR}/src/sift_line_endpoint.cpp
  ${PROJECT_SOURCE_DIR}/src/initialize_mono.cpp
  ${PROJECT_SOURCE_DIR}/src/mappoint.cpp
  ${PROJECT_SOURCE_DIR}/src/camera.cpp
)

set(3rd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/3rd_party/LSDDetector_custom.cpp
  ${PROJECT_SOURCE_DIR}/3rd_party/DBoW2/DUtils/Random.cpp
  ${PROJECT_SOURCE_DIR}/3rd_party/DBoW2/DUtils/Timestamp.cpp
)


FILE(GLOB SOPHUS_LIST_CPP ${PROJECT_SOURCE_DIR}/3rd_party/sophus/*.cpp)
#或者用下列句子代替?
#FILE(GlOB SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/3rd_party/include
  ${PROJECT_SOURCE_DIR}/3rd_party/sophus
  ${PROJECT_SOURCE_DIR}/3rd_party/DBoW2/DUtils
  ${PROJECT_SOURCE_DIR}/include
) 

add_library(${PROJECT_NAME} ${SOPHUS_LIST_CPP} ${3rd_SOURCE_FILES} ${SLEmonocular_SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
add_executable(test_vedio ${PROJECT_SOURCE_DIR}/test/test_vedio.cpp )
target_link_libraries(test_vedio ${PROJECT_NAME})

#configure_file(${PROJECT_SOURCE_DIR}/config/default.yaml ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
#configure_file(${PROJECT_SOURCE_DIR}/vedio/test_color.mp4 ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})



