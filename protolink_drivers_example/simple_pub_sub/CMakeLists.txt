cmake_minimum_required(VERSION 3.8)
project(simple_pub_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(protolink REQUIRED)

add_protolink_message_from_ros_message("geometry_msgs" "Vector3")

add_executable(simple_pub_sub
  ./src/simple_pub_sub.cpp
)
ament_target_dependencies(simple_pub_sub
  rclcpp
  geometry_msgs
  protolink
)
target_link_libraries(simple_pub_sub
  geometry_msgs__Vector3_proto
)

install(TARGETS simple_pub_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ./launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
