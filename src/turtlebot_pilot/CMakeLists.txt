cmake_minimum_required(VERSION 3.8)
project(turtlebot_pilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(PROGRAMS
    src/navigation/navigator.py
    src/navigation/explorer.py
    src/navigation/explorer_stop.py
    src/navigation/frontier_explore.py
    src/control/heading_controller.py
    src/perception/perception_controller.py
    src/perception/icp_node.py
    src/perception/icp_utils.py
    src/utils/p3_plot.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
        DESTINATION share/${PROJECT_NAME})

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

ament_package()
