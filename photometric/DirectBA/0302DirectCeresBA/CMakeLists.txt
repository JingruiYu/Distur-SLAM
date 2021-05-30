cmake_minimum_required(VERSION 2.8)

project(directCeresBA)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(directCeresBA directCeresBA.cpp)

target_link_libraries(directCeresBA ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
