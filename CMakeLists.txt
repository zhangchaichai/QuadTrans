cmake_minimum_required(VERSION 3.17)
project(QuadTrans)

set(CMAKE_CXX_STANDARD 14)

add_executable(QuadTrans main.cpp JsonHelper.cpp JsonHelper.h DetectorState.cpp DetectorState.h Vector2.h Box.h Quadtree.h Point.h "B+ tree.h" B+_insertion.cpp B+_display.cpp B+_search.cpp utiFunc.cpp)
#link_directories(/home/zcc/vcpkg/packages/jsoncpp_x64-linux/lib)
#include_directories(/home/zcc/vcpkg/packages/jsoncpp_x64-linux/include)
find_package(jsoncpp CONFIG REQUIRED)
target_link_libraries(QuadTrans jsoncpp)