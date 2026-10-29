cmake_minimum_required(VERSION 3.8)
project(ias0220_243483)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  ias0220_243483/odometry.py
  ias0220_243483/steering_node.py
  DESTINATION lib/${PROJECT_NAME})


# Task 4: install launch files, MuJoCo model, RViz config and data so
# get_package_share_directory() can find them after a build
install(DIRECTORY launch mujoco config
  DESTINATION share/${PROJECT_NAME})

# Task 5.2: bags used by the launch files (found via get_package_share_directory).
# rosbag_to_X only exists after recording it in the lab, so install it only if present.
foreach(bag record2 rosbag_to_X)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/bags/${bag})
    install(DIRECTORY bags/${bag} DESTINATION share/${PROJECT_NAME}/bags)
  endif()
endforeach()

ament_package()
