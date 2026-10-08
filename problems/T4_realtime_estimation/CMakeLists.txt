cmake_minimum_required(VERSION 3.22)
project(t4_realtime_estimation LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SignalPacket.msg"
    DEPENDENCIES std_msgs
)

add_executable(estimator_node src/estimator_node.cpp)
target_compile_options(estimator_node PRIVATE -Wall -Wextra -Wpedantic)
# 直接运行构建目录中的节点时，ROS 类型支持库的 dlopen 也需要构建 RPATH。
target_link_options(estimator_node PRIVATE "-Wl,--disable-new-dtags")
ament_target_dependencies(estimator_node
    rclcpp
    rcl_interfaces
    std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(estimator_node "${cpp_typesupport_target}")
add_dependencies(estimator_node
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
    ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
)

install(TARGETS
    estimator_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
