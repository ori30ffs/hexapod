cmake_minimum_required(VERSION 2.8.3)
project(ros_hex_legs)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs image_transport hexa_msgs vrep_common)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(vrep_hexapod_simulator src/rosHexapod.cpp) 
target_link_libraries(vrep_hexapod_simulator ${catkin_LIBRARIES})
add_dependencies(vrep_hexapod_simulator vrep_common_generate_messages_cpp)

