cmake_minimum_required(VERSION 2.8.11)

project(bvs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(APPLE)
    set(OpenCV_DIR /usr/local/Cellar/opencv3/3.3.0_2/share/OpenCV)
endif()

find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS}
                    /usr/local/include/sparsehash
                    /usr/local/include/eigen3)

####################### FILES
set(MAXFLOW_FILES
    maxflow-v3.01/maxflow.cpp
    maxflow-v3.01/graph.cpp)

set(BFILT_FILES
    BilateralFilter.cpp)

set(BSEG_FILES
    BilateralSegment.cpp
    ${MAXFLOW_FILES})

set(GRIDTEST_FILES
    TestGrid.cpp)

#################### LIBS

set(COMMON_LIBS
    ${OpenCV_LIBRARIES})

###################### CREATE TARGETS

add_executable(bfilt
               ${BFILT_FILES})

add_executable(bseg
               ${BSEG_FILES})

add_executable(gridtest
               ${GRIDTEST_FILES})

target_link_libraries(bfilt
                      ${COMMON_LIBS})

target_link_libraries(bseg
                      ${COMMON_LIBS})

target_link_libraries(gridtest
                      ${COMMON_LIBS})
