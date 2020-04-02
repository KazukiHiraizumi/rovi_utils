cmake_minimum_required(VERSION 2.8.3)
project(rovi_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  rovi
)

add_service_files(
  FILES
  TextFilter.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions("-std=c++11")

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_executable(floats3pc src/floats3pc.cpp json11/json11.cpp)
#target_link_libraries(floats3pc ${catkin_LIBRARIES})
