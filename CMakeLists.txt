cmake_minimum_required(VERSION 2.8)

project(pcl_visualizer)
set(CMAKE_BUILD_TYPE Relese)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

MESSAGE(STATUS "This is BINARY dir " ${pcl_visualizer_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${pcl_visualizer_SOURCE_DIR})

FIND_PACKAGE(PCL 1.7 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OpenGL_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenGL_LIBRARY_DIRS})

FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OpenGL_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenGL_LIBRARY_DIRS})

FIND_PACKAGE(GLUT REQUIRED)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIRS})
LINK_DIRECTORIES(${GLUT_LIBRARY_DIRS})

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(FMT REQUIRED)
INCLUDE_DIRECTORIES(${FMT_INCLUDE_DIRS})

FIND_PACKAGE(Boost REQUIRED COMPONENTS
#                            chrono
#                            filesystem
#                            thread
#                            system)
                                   )
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(SRC_PATH src)
ADD_SUBDIRECTORY(${SRC_PATH})
#set(SRC_LIST ${SRC_PATH}/visualizer.cc
#             ${SRC_PATH}/run_visual.cc)

AUX_SOURCE_DIRECTORY(src/visualizer VISUAL_SRC)
AUX_SOURCE_DIRECTORY(src/bin2pcd BIN2PCD_SRC)
AUX_SOURCE_DIRECTORY(src/disparity2pcl Depth2Pcl_SRC)

set(INC_PATH inc)
INCLUDE_DIRECTORIES(${INC_PATH})

SET(visualizer_LIBS ${Boost_LIBRARIES}
                    ${PCL_LIBRARIES}
                    ${OpenGL_LIBRARIES}
                    ${GLUT_LIBRARY}
                    glog
                    fmt::fmt)

SET(bin2pcd_LIBS ${PCL_LIBRARIES}
                 glog
                 fmt::fmt)

SET(Depth2Pcl_LIBS ${Boost_LIBRARIES}
                    ${PCL_LIBRARIES}
                    ${OpenCV_LIBS}
                    glog
                    fmt::fmt)

#LINK_LIBRARIES(#${OpenCV_LIBS}
#               ${Boost_LIBRARIES}
#               ${PCL_LIBRARIES}
#               glog
#               fmt::fmt)

add_executable(pcl_visualizer ${VISUAL_SRC})
target_link_libraries(pcl_visualizer ${visualizer_LIBS})


add_executable(bin_to_pcd ${BIN2PCD_SRC})
target_link_libraries(bin_to_pcd ${bin2pcd_LIBS})

add_executable(depth_to_pcl ${Depth2Pcl_SRC})
target_link_libraries(depth_to_pcl ${Depth2Pcl_LIBS})
