cmake_minimum_required(VERSION 3.5)
project(aprs_gz_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclpy
  rclcpp
  aprs_interfaces
  ros_gz_interfaces
)

find_package(gz-transport13 REQUIRED)

find_package(gz-msgs10 REQUIRED)

# set(GZ_TARGET_PREFIX gz)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  models
  gz_models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

add_executable(spawn_part src/spawn_part.cpp)
target_include_directories(spawn_part PRIVATE include)

target_link_libraries(spawn_part
  gz-msgs10::core
  gz-transport13::core
)

ament_target_dependencies(spawn_part ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS spawn_part
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/environment_startup_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()