cmake_minimum_required(VERSION 3.8)
project(qt_ros_nav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  rviz
  img_detect
  cv_bridge      
  image_transport
)
find_package(Qt5 COMPONENTS Widgets Core Gui SerialPort REQUIRED)

catkin_package(
  INCLUDE_DIRS include/qt_ros_nav
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs std_srvs rviz img_detect
)

include_directories(
  include/qt_ros_nav
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(QT_SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/ros_node.cpp
  src/serial_comm.cpp
)
set(QT_HEADERS
  include/qt_ros_nav/mainwindow.h
  include/qt_ros_nav/ros_node.h
  include/qt_ros_nav/serial_comm.h
)
set(QT_FORMS
  ui/mainwindow.ui
)
set(QT_RESOURCES
  resources/icons.qrc
)

add_executable(${PROJECT_NAME}
  ${QT_SOURCES}
  ${QT_HEADERS}
  ${QT_FORMS}
  ${QT_RESOURCES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::SerialPort
)
