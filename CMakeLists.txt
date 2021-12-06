cmake_minimum_required(VERSION 3.7..3.22)
project( nbody VERSION 0.1
    DESCRIPTION "Basic Nbody simulator"
    LANGUAGES CXX)

#find_package(OpenCV PATHS /usr/local/lib/cmake NO_DEFAULT_PATH)
find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(nbody nbody.cpp include/MassBody.h src/MassBody.cpp)

target_include_directories( nbody PUBLIC include )
#add_executable( MassBody src/MassBody.cpp )
#target_link_libraries( MassBody ${OpenCV_LIBS})


target_link_libraries( nbody ${OpenCV_LIBS})
