cmake_minimum_required(VERSION 3.8)
project(aura_sim_dashboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# --- ADDED INSTALLATION RULES ---

# 1. Install launch and web directories to the package's share folder
install(DIRECTORY 
  launch
  web
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install python scripts to the lib folder so they can be executed via 'ros2 run'
# USE_SOURCE_PERMISSIONS ensures they remain executable (chmod +x)
install(DIRECTORY 
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# --------------------------------

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
