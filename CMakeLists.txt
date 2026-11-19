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
  ias0220_243483/image_publisher.py
  ias0220_243483/camera_calibration.py
  ias0220_243483/object_recognition.py
  ias0220_243483/simple_control.py
  ias0220_243483/visual_servoing.py
  DESTINATION lib/${PROJECT_NAME})


# Install launch files, MuJoCo model (kept for Task 6.2) and RViz config so
# get_package_share_directory() can find them after a build
install(DIRECTORY launch mujoco config
  DESTINATION share/${PROJECT_NAME})

# Task 6.1: calibration images (data/images). ament_cmake has no setup.py
# data_files, so the data folder is installed here instead.
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME})

ament_package()
