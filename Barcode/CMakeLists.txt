SET(PROJECT_NAME Barcode)  

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

find_package(OpenCV 3.0 QUIET)

if(OpenCV_FOUND)
  message("Find_OpenCV")
else()
  message(FATAL_ERROR "Not Find OpenCV")
endif()


# Sources files
set(Barcode_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/barcode_localization.cpp
  ${PROJECT_SOURCE_DIR}/src/extrafunction.cpp
  ${PROJECT_SOURCE_DIR}/src/find_barcode.cpp
  ${PROJECT_SOURCE_DIR}/src/solve_pnp.cpp
)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OPENCV_INCLUDE_DIR}
) 

add_library(${PROJECT_NAME} ${Barcode_SOURCE_FILES} )

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin/example)
add_executable(test_barcode ${PROJECT_SOURCE_DIR}/examples/test_barcode.cpp )
target_link_libraries(test_barcode ${PROJECT_NAME})




