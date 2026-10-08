cmake_minimum_required(VERSION 3.16)
project(so_arm_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(so_arm_grasp_plugin SHARED src/grasp_plugin.cpp)
target_compile_features(so_arm_grasp_plugin PUBLIC cxx_std_17)
target_compile_options(so_arm_grasp_plugin PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(so_arm_grasp_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(so_arm_grasp_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(so_arm_grasp_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(so_arm_grasp_plugin gazebo_ros rclcpp std_srvs)

ament_python_install_package(${PROJECT_NAME})
install(TARGETS so_arm_grasp_plugin LIBRARY DESTINATION lib)
install(PROGRAMS scripts/pick_and_place scripts/fk_learning scripts/fk_dashboard
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch meshes urdf worlds DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(kinematics test/test_kinematics.py)
  ament_add_pytest_test(fk_learning test/test_fk_learning.py)
endif()

ament_package()
