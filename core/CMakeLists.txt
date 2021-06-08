cmake_minimum_required(VERSION 2.8)
project(Distur)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

# include_directories( "/usr/include/eigen3" )

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/line_descriptor)

include_directories(
    include
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS} 
)

add_library(${PROJECT_NAME}
    src/readData.cpp
    src/disSLAM.cpp
    src/Frame.cpp
    src/tracking.cpp
    src/map.cpp
    src/poseSolver.cpp
    src/view.cpp
    src/convert.cpp
    src/SE2.cpp
    src/keyFrame.cpp
    src/optimizer.cpp
    src/lineport.cpp
    src/LineExtractor.cpp
    src/KeyLineGeometry.cpp
    src/FeatureLine.cpp
    src/config.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
pangolin
${CERES_LIBRARIES}
linedesc
DBoW2
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

add_executable(try test/try.cpp)
target_link_libraries(try ${PROJECT_NAME})

add_executable(main test/main.cpp)
target_link_libraries(main ${PROJECT_NAME})