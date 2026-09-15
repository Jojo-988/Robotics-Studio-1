cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

install(
  DIRECTORY config launch models urdf_husky worlds urdf_parrot
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dynamic_world_demo.py 
           scripts/basic_autonomy_demo.py 
           scripts/fire_hotspot_mission.py 
           scripts/parrot_lidar_3d_mapping.py
           scripts/parrot_mapping_mission.py
           scripts/large_environment_demo.py
           scripts/vegetation_detector.py
           scripts/fire_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
