cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17 -O2")

project(EA_IPM)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS}
    include
)


add_library(${PROJECT_NAME} SHARED
    src/IPMManager.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})

