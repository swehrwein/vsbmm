cmake_minimum_required(VERSION 2.8.11)

project(vidseg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(OpenCV_DIR /usr/local/Cellar/opencv3/3.3.0_2/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(Theia REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${THEIA_INCLUDE_DIRS}
                    ../common)

####################### FILES

set(MOTIONMODEL_FILES
    Instantiation.cpp
    MotionModel.cpp
    Homography.cpp)

set(TRACKADAPTER_FILES
    Instantiation.cpp
    TrackAdapter.cpp)

set(TRACKRANSAC_FILES
    TrackVis.cpp
    TrackRansac.cpp)

#################### LIBS

set(COMMON_LIBS
    ${THEIA_LIBRARIES}
    ${OpenCV_LIBRARIES})

###################### CREATE TARGETS

add_library(TrackMotion
            ${MOTIONMODEL_FILES})

add_executable(trackadapter ${TRACKADAPTER_FILES})
add_executable(trackransac ${TRACKRANSAC_FILES})

target_link_libraries(trackadapter
                      ${COMMON_LIBS})

target_link_libraries(trackransac
                      TrackMotion
                      ${COMMON_LIBS})

