cmake_minimum_required(VERSION 3.8)
project(string_pub_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(protolink REQUIRED)

add_protolink_message_from_ros_message("std_msgs" "String")

add_executable(string_pub_sub
  ./src/string_pub_sub.cpp
)
ament_target_dependencies(string_pub_sub
  rclcpp
  std_msgs
  protolink
)
target_link_libraries(string_pub_sub
  std_msgs__String_proto
)

install(TARGETS string_pub_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ./launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
