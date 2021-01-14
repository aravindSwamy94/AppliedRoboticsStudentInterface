cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )

find_package(project_interface REQUIRED )

find_package(Boost REQUIRED program_options atomic)
find_package(Eigen3)
find_package(ompl REQUIRED)
## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
 ${CUSTOM_EIGEN}
 ${CMAKE_CURRENT_SOURCE_DIR}/src/RRTStar/
)

add_executable(calibrate src/camera_calibration.cpp)
target_link_libraries(calibrate ${OpenCV_LIBRARIES})
## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
   src/extrisnicCalib.cpp
   src/findRobot.cpp
   src/processMap.cpp
   src/loadImage.cpp
   src/clipper.cpp
   src/planPath.cpp
   src/RRTStar/obstacles.cpp  
   src/RRTStar/rrtstar.cpp 
)
add_library(dubins SHARED
  src/dubins_local.cpp
)

list(APPEND CLOTHOIDS_INCLUDE_DIRS "/home/aravind/Desktop/Trento_Exit/RPA/Clothoids_install/Clothoids/submodules/Utils/src/")
list(APPEND CLOTHOIDS_INCLUDE_DIRS "/home/aravind/Desktop/Trento_Exit/RPA/Clothoids_install/Clothoids/src/")
list(APPEND CLOTHOIDS_INCLUDE_DIRS "/home/aravind/Desktop/Trento_Exit/RPA/Clothoids_install/Clothoids/lib/include/")

add_library(Clothoids SHARED IMPORTED)
set_target_properties(Clothoids PROPERTIES
  IMPORTED_LOCATION "/home/aravind/Desktop/Trento_Exit/RPA/Clothoids_install/Clothoids/lib/lib/libClothoids_linux.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CLOTHOIDS_INCLUDE_DIRS}"
)

list(APPEND CONFIG4CPP_INCLUDE_DIRS "/home/aravind/Desktop/Trento_Exit/RPA/config4cpp/include")
add_library (Config4cpp SHARED IMPORTED)
set_target_properties(Config4cpp PROPERTIES
  IMPORTED_LOCATION "/home/aravind/Desktop/Trento_Exit/RPA/config4cpp/lib/libconfig4cpp.a"
  INTERFACE_INCLUDE_DIRECTORIES "${CONFIG4CPP_INCLUDE_DIRS}"
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}
  tesseract  
  dubins
  ${OMPL_LIBRARIES}
  Clothoids
  Config4cpp
)


#if(Eigen3_FOUND)
#  set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
#else()
# find_package(Eigen)
#  set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
#endif()


## COMPILE DEMO EXECUTABLE



# Demo on Perception Algorithms
#add_executable(demo_dilatation_erosion
#  src/demo_shape_detection/demo_dilatation_erosion.cpp
#)
#add_executable(demo_hsv_filter
#  src/demo_shape_detection/demo_hsv_filter.cpp
#)
#add_executable(demo_smoothing
#  src/demo_shape_detection/demo_smoothing.cpp
#)
#add_executable(demo_filters
#  src/demo_shape_detection/demo_filters.cpp
#)
#add_executable(demo_rgb_filter
#  src/demo_shape_detection/demo_rgb_filter.cpp
#)
#add_executable(demo_full_example
#  src/demo_shape_detection/demo_full_example.cpp
#)
#add_executable(demo_simple_shape_detection
#  src/demo_shape_detection/demo_simple_shape_detection.cpp
#)

#target_link_libraries(demo_dilatation_erosion
#  ${OpenCV_LIBRARIES}
#)
#target_link_libraries(demo_hsv_filter
#  ${OpenCV_LIBRARIES}
#)
#target_link_libraries(demo_smoothing
#  ${OpenCV_LIBRARIES}
#)
#target_link_libraries(demo_filters
#  ${OpenCV_LIBRARIES}
#)
#target_link_libraries(demo_rgb_filter
#  ${OpenCV_LIBRARIES}
#)
#target_link_libraries(demo_full_example
#  ${OpenCV_LIBRARIES}
#)
#target_link_libraries(demo_simple_shape_detection
#  ${OpenCV_LIBRARIES}
#)



# DEMOS on Path Planning

#add_executable(test_dubins
#  src/test_dubins_local.cpp
#)
#target_link_libraries(test_dubins dubins)



