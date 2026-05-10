cmake_minimum_required(VERSION 3.8)
project(ros2wifibot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  DEPENDENCIES std_msgs
)

add_library(libwifibot STATIC src/libwifibot.cpp)
target_include_directories(libwifibot PUBLIC include)

add_executable(wifibot_node src/wifibot.cpp)

# ✅ plain signature — pas de mot-clé PRIVATE
ament_target_dependencies(wifibot_node
  rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
)

# ✅ plain signature cohérente avec ament_target_dependencies
target_link_libraries(wifibot_node libwifibot)

# ✅ API moderne pour les messages du même package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(wifibot_node "${cpp_typesupport_target}")

install(TARGETS wifibot_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()