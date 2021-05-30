cmake_minimum_required( VERSION 2.8 )
project( directMethod )

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories( "/usr/include/eigen3" )

# add_executable( semanticDB semanticDB.cpp )
# target_link_libraries( semanticDB 
# ${OpenCV_LIBS}  
# /home/yujr/mono_brid_semantic/Thirdparty/g2o/lib/libg2o.so
# )

# add_executable( direct_sparse direct_sparse.cpp )
# target_link_libraries( direct_sparse 
# ${OpenCV_LIBS}  
# /home/yujr/mono_brid_semantic/Thirdparty/g2o/lib/libg2o.so
# )

add_executable( try try.cpp )
target_link_libraries( try 
${OpenCV_LIBS}  
/home/yujr/mono_brid_semantic/Thirdparty/g2o/lib/libg2o.so
)