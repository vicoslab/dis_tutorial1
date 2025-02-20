cmake_minimum_required(VERSION 3.5)
project(dis_tutorial1)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Always find ament_cmake
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Find all the packages that we are using
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# This is needed if we generate custom interfaces (messages, services or actions)
find_package(rosidl_default_generators REQUIRED)

# A macro, not to repeat the dependencies
set(dependencies rclcpp std_msgs std_srvs rosidl_default_generators)

# We list the interface files and their dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "srv/AddTwoInts.srv"
  DEPENDENCIES std_msgs # Add packages that the listed interfaces depend on
)

# We load the exectutables and give each the dependencies
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher ${dependencies})

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber ${dependencies})

add_executable(simple_client_node src/simple_client_node.cpp)
ament_target_dependencies(simple_client_node ${dependencies})

add_executable(simple_server_node src/simple_server_node.cpp)
ament_target_dependencies(simple_server_node ${dependencies})

add_executable(complete_node src/complete_node.cpp)
ament_target_dependencies(complete_node ${dependencies})

add_executable(simple_set_bool_server_node src/simple_set_bool_server_node.cpp)
ament_target_dependencies(simple_set_bool_server_node ${dependencies})

# Finally, instructions for installing in the appropriate output folder
install(TARGETS
    simple_publisher
    simple_subscriber
    simple_client_node
    simple_server_node
    complete_node
    simple_set_bool_server_node
    DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/py_simple_publisher.py
  scripts/py_simple_subscriber.py
  scripts/py_simple_server_node.py
  scripts/py_simple_client_node.py
  scripts/py_complete_node.py
  scripts/py_draw_square.py
  DESTINATION lib/${PROJECT_NAME}
)

# This is needed if our exported targets depend on interfaces built in the package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(simple_client_node "${cpp_typesupport_target}")
target_link_libraries(simple_server_node "${cpp_typesupport_target}")
target_link_libraries(complete_node "${cpp_typesupport_target}")

ament_export_dependencies(${dependencies} rosidl_default_runtime)
ament_package()
