cmake_minimum_required(VERSION 3.8)
project(tb4_cpp_prac1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(irobot_create_msgs REQUIRED)

# Additional Package Dependency
find_package(std_srvs REQUIRED)

add_executable(tb4_cpp_node src/tb4_cpp_node.cpp)
ament_target_dependencies(tb4_cpp_node rclcpp irobot_create_msgs)

# Additional node and its dependency
add_executable(tb4_cpp_srv_server src/tb4_cpp_srv_server.cpp)
ament_target_dependencies(tb4_cpp_srv_server rclcpp irobot_create_msgs std_srvs)

target_include_directories(tb4_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(tb4_cpp_srv_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(tb4_cpp_node PUBLIC c_std_99 cxx_std_17)

# Additional Node Installation
install(TARGETS tb4_cpp_node tb4_cpp_srv_server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()