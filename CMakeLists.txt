cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(view_shapenet_partseg)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

# Set cmake prefix path to enable cmake to find freenect2
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
#find_package(freenect2 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
#find_package(Boost REQUIRED)
#find_package(OpenMesh REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

#include_directories("/usr/include/libusb-1.0/")
#include_directories(${freenect2_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${OpenMesh_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
#link_directories(${OpenMesh_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})
#add_definitions(${OpenMesh_DEFINITIONS})

# YAMLCPP
# attempt to find static library first if this is set
if(YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif()

# find the yaml-cpp include directory
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
        PATH_SUFFIXES include
        PATHS
        ~/Library/Frameworks/yaml-cpp/include/
        /Library/Frameworks/yaml-cpp/include/
        /usr/local/include/
        /usr/include/
        /sw/yaml-cpp/         # Fink
        /opt/local/yaml-cpp/  # DarwinPorts
        /opt/csw/yaml-cpp/    # Blastwave
        /opt/yaml-cpp/
        ${YAMLCPP_DIR}/include/)

# find the yaml-cpp library
find_library(YAMLCPP_LIBRARY
        NAMES ${YAMLCPP_STATIC} yaml-cpp
        PATH_SUFFIXES lib64 lib
        PATHS ~/Library/Frameworks
        /Library/Frameworks
        /usr/local
        /usr
        /sw
        /opt/local
        /opt/csw
        /opt
        ${YAMLCPP_DIR}/lib)
# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(YAMLCPP DEFAULT_MSG YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)
mark_as_advanced(YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)


add_executable (view_shapenet_partseg src/view_shapenet-partseg_main.cpp)
target_link_libraries (view_shapenet_partseg ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (create_shapenet_partseg_from_linemod src/create_shapenet_partseg_from_linemod_main.cpp)
target_link_libraries (create_shapenet_partseg_from_linemod ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (create_shapenet_partseg_from_linemod_test src/create_shapenet_partseg_from_linemod_test_main.cpp)
target_link_libraries (create_shapenet_partseg_from_linemod_test ${PCL_LIBRARIES} ${OpenCV_LIBS} ${YAMLCPP_LIBRARY})

add_executable (create_shapenet_partseg_from_linemod_test_02 src/create_shapenet_partseg_from_linemod_test_02_main.cpp)
target_link_libraries (create_shapenet_partseg_from_linemod_test_02 ${PCL_LIBRARIES} ${OpenCV_LIBS} ${YAMLCPP_LIBRARY})

add_executable (test src/test.cpp)
target_link_libraries (test ${PCL_LIBRARIES} ${OpenCV_LIBS} ${YAMLCPP_LIBRARY})

add_executable (read_gt_to_matrix src/read_gt_to_matrix_main.cpp)
target_link_libraries (read_gt_to_matrix ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (view_gt_of_02_as_shapenet_partseg src/view_gt_of_02_as_shapenet_partseg.cpp)
target_link_libraries (view_gt_of_02_as_shapenet_partseg ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (view_test src/view_test_main.cpp)
target_link_libraries (view_test ${PCL_LIBRARIES} ${OpenCV_LIBS})