cmake_minimum_required(VERSION 2.8)
project(Distur)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(OpenCV QUIET)
# find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
    src/readData.cpp
    src/disSLAM.cpp
    src/Frame.cpp
    src/tracking.cpp
    src/map.cpp
    src/poseSolver.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
# ${EIGEN3_LIBS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

add_executable(main test/main.cpp)
target_link_libraries(main ${PROJECT_NAME})