cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++0x")
project( use_kitti)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )


find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(Sophus)

include_directories(${Sophus_INCLUDE_DIRS})

include_directories(/usr/include/eigen3/Eigen)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Threads)
include_directories(${Threads_INCLUDE_DIRS})

add_library(use_kitti_lib SHARED
        ${PROJECT_SOURCE_DIR}/src/opticalflow.cpp
        ${PROJECT_SOURCE_DIR}/src/DrawTrajectory.cpp
        ${PROJECT_SOURCE_DIR}/src/inpute.cpp
        ${PROJECT_SOURCE_DIR}/src/poseestimate_2d2d.cpp
        ${PROJECT_SOURCE_DIR}/src/camare.cpp
        ${PROJECT_SOURCE_DIR}/src/triangulation.cpp

        )


add_executable(${PROJECT_NAME} use_kitti.cpp)

target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_lib}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        use_kitti_lib
        )

include_directories(${PROJECT_SOURCE_DIR}/include)
#add_subdirectory(src)
