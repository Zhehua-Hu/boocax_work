SET(PROJECT_NAME Ekf_p4p)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(BUILD_TYPE Release)
MESSAGE("BUild type:" ${BUILD_TYPE})

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# required libraries
find_package(Pangolin QUIET)
find_package(OpenCV 3.0 QUIET)

if(Pangolin_FOUND)
 message("Find_Pangolin")
else()
  message(FATAL_ERROR "Not Find Pangolin")
endif()

if(OpenCV_FOUND)
  message("Find_OpenCV")
else()
  message(FATAL_ERROR "Not Find OpenCV")
endif()


# Sources files
set(EKFP4P_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/camera.cpp
  ${PROJECT_SOURCE_DIR}/src/extra_function.cpp
  ${PROJECT_SOURCE_DIR}/src/system.cpp
  ${PROJECT_SOURCE_DIR}/src/vehicle.cpp
  ${PROJECT_SOURCE_DIR}/src/viewer.cpp
)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Pangolin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIR}
) 

add_library(${PROJECT_NAME} ${EKFP4P_SOURCE_FILES} )

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
add_executable(main main.cpp )
target_link_libraries(main ${PROJECT_NAME})




