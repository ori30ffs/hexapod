cmake_minimum_required(VERSION 2.8.3)
project(hexa_kinematics)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs image_transport vrep_common hexa_msgs)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(hexa_kinematics_server src/kinematics.cpp) 
target_link_libraries(hexa_kinematics_server ${catkin_LIBRARIES})
add_dependencies(hexa_kinematics_server vrep_common_generate_messages_cpp)
