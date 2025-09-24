cmake_minimum_required(VERSION 3.0.2)
project(set_mass_plugin)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros gazebo_msgs)

find_package(gazebo REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(set_mass_plugin SHARED src/SetMassPlugin.cpp)
target_link_libraries(set_mass_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


install(TARGETS set_mass_plugin DESTINATION $ENV{LOCOSIM_DIR}/robot_descriptions/gazebo_plugins/lib)
