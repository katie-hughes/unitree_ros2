cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


message(STATUS "-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)


set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/include
)

install(
 DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/lib/
 DESTINATION lib
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/lib)

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(udp_high src/udp_high.cpp)
target_link_libraries(udp_high ${EXTRA_LIBS})
ament_target_dependencies(udp_high rclcpp ros2_unitree_legged_msgs)

add_executable(jsp_high src/jsp_high.cpp)
target_link_libraries(jsp_high ${EXTRA_LIBS})
ament_target_dependencies(jsp_high rclcpp ros2_unitree_legged_msgs sensor_msgs)

add_executable(jsp_low src/jsp_low.cpp)
target_link_libraries(jsp_low ${EXTRA_LIBS})
ament_target_dependencies(jsp_low rclcpp ros2_unitree_legged_msgs sensor_msgs)

add_executable(udp_low src/udp_low.cpp)
target_link_libraries(udp_low ${EXTRA_LIBS})
ament_target_dependencies(udp_low rclcpp ros2_unitree_legged_msgs sensor_msgs)

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

add_executable(custom_gait src/custom_gait.cpp)
target_link_libraries(custom_gait ${EXTRA_LIBS})
ament_target_dependencies(custom_gait rclcpp ros2_unitree_legged_msgs)

install(TARGETS
    ros2_udp
    udp_high
    jsp_high
    ros2_walk_example
    ros2_position_example
    custom_gait
    udp_low
    jsp_low
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
