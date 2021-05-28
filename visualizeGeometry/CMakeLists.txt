cmake_minimum_required( VERSION 2.8 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++11")

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )

add_executable( testVisual testVisual.cpp )
target_link_libraries( testVisual ${Pangolin_LIBRARIES} )

# add_executable( testFitLine testFitLine.cpp )
# target_link_libraries( testFitLine ${Pangolin_LIBRARIES} )