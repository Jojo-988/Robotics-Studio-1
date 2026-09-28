cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(patrol)

find_package(ros_ign_interfaces REQUIRED)

install(
  DIRECTORY config launch models urdf_husky worlds urdf_parrot docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dynamic_world_demo.py scripts/basic_autonomy_demo.py scripts/coverage_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(coverage_patrol_tests test/test_coverage_patrol.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(patrol_ros_tests test/test_patrol_ros.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=169)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
