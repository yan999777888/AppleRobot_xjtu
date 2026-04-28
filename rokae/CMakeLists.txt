cmake_minimum_required(VERSION 3.12) #版本高，可能存在问题
project(rokae)

#------------------------------------------------------------------------------
## General settings

# Compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Using static/shared libraries option
option(XCORE_LINK_SHARED_LIBS "Example executables link shared library" OFF)
option(XCORE_USE_XMATE_MODEL "The library of kinematics and dynamics calculation for cobot" OFF)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  task_assign
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external  
  ${PCL_INCLUDE_DIRS}
)
# External resources/repositories are downloaded here
# Third-party dependencies

# Header-only library Eigen
add_library(eigen INTERFACE)
target_compile_definitions(eigen INTERFACE EIGEN_DONT_ALIGN_STATICALLY)
# target_include_directories(eigen INTERFACE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>)

# # Public headers
# set(SHARED_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Here puts required headers
set(SHARED_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/data_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/exception.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/force_control.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/motion_control_rt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/robot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rokae/utility.h
    )

# set(SHARED_HEADERS ${SHARED_HEADERS} CACHE INTERNAL "Shared headers with examples")

# xCore-SDK libraries
# Add shared/static libraries
add_library(${PROJECT_NAME}_lib SHARED IMPORTED GLOBAL)
add_library(${PROJECT_NAME}_static STATIC IMPORTED GLOBAL)
add_library(xmatemodel_lib STATIC IMPORTED)

set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        INTERFACE_LINK_LIBRARIES xmatemodel_lib)

# set path to model library
set_target_properties(xmatemodel_lib PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libxMateModel.a)

set(XCORESDK_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
    IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}.so.${CMAKE_PROJECT_VERSION}
    )
set_target_properties(${PROJECT_NAME}_static PROPERTIES
    IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.a
    INTERFACE_LINK_LIBRARIES xmatemodel_lib
    )

# Add library alias
if(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS ${PROJECT_NAME}_lib)
else()
    add_library(Rokae::Rokae ALIAS ${PROJECT_NAME}_static)
endif()

# Install runtime artifacts
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.21")
    install(IMPORTED_RUNTIME_ARTIFACTS ${PROJECT_NAME}_lib
        RUNTIME
        DESTINATION ${CMAKE_INSTALL_BINDIR})
else()
    string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type_upper)
    get_target_property(runtime_dll_location ${PROJECT_NAME}_lib IMPORTED_LOCATION_${build_type_upper})
    if(${runtime_dll_location} MATCHES "NOTFOUND$")
        get_target_property(runtime_dll_location ${PROJECT_NAME}_lib IMPORTED_LOCATION)
    endif()
    if(NOT ${runtime_dll_location} MATCHES "NOTFOUND$")
        install(FILES ${runtime_dll_location} DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
endif()

# Example usages
add_compile_definitions(
    # for M_PI in cmath
    $<$<CXX_COMPILER_ID:MSVC>:_USE_MATH_DEFINES>
    # if linking the shared library, add a macro for MSVC dllimport attribute
    $<$<BOOL:${XCORE_LINK_SHARED_LIBS}>:XCORESDK_DLL>
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/print_helper.hpp
)

find_package (Threads REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/test.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/test_node.cpp)
add_executable(leftRokae src/leftRokae.cpp ${HEADERS})
add_executable(leftRokaegoHome src/leftRokaegoHome.cpp ${HEADERS})
add_executable(leftRokaegodemo src/leftRokaegodemo.cpp ${HEADERS})
add_executable(rightRokae src/rightRokae.cpp ${HEADERS})
add_executable(smart_robot_node src/smart_robot_node.cpp ${HEADERS})
add_executable(right_robot_control_node src/right_robot_control_node.cpp ${HEADERS})
add_executable(left_robot_control_node src/left_robot_control_node.cpp ${HEADERS})
add_executable(auto_handeye_calibration src/auto_handeye_calibration.cpp ${HEADERS})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(leftRokae ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(leftRokaegoHome ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(leftRokaegodemo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rightRokae ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(smart_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(right_robot_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(left_robot_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(auto_handeye_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(leftRokae 
  ${catkin_LIBRARIES}
  Rokae::Rokae 
  Threads::Threads 
  eigen
)

target_link_libraries(leftRokaegoHome
  ${catkin_LIBRARIES}
  Rokae::Rokae 
  Threads::Threads 
  eigen
)

target_link_libraries(leftRokaegodemo
  ${catkin_LIBRARIES}
  Rokae::Rokae 
  Threads::Threads 
  eigen
)

target_link_libraries(rightRokae 
  ${catkin_LIBRARIES}
  Rokae::Rokae 
  Threads::Threads 
  eigen
)

target_link_libraries(smart_robot_node
  ${catkin_LIBRARIES}
  Rokae::Rokae 
  Threads::Threads 
  eigen
  ${PCL_LIBRARIES}
)


target_link_libraries(right_robot_control_node
  ${catkin_LIBRARIES}
  Rokae::Rokae
  Threads::Threads
  eigen
  ${PCL_LIBRARIES}
)

target_link_libraries(left_robot_control_node
  ${catkin_LIBRARIES}
  Rokae::Rokae
  Threads::Threads
  eigen
  ${PCL_LIBRARIES}
)

target_link_libraries(auto_handeye_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Rokae::Rokae
  Threads::Threads
  eigen
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
