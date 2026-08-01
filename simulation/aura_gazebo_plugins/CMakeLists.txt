cmake_minimum_required(VERSION 3.8)
project(aura_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Plugin: kawal warna traffic light (merah -> kuning -> hijau, auto cycle)
add_library(traffic_light_plugin SHARED src/traffic_light_plugin.cpp)
target_link_libraries(traffic_light_plugin ${GAZEBO_LIBRARIES})

# Plugin: kawal pergerakan traffic bar (boom gate naik/turun)
add_library(traffic_bar_plugin SHARED src/traffic_bar_plugin.cpp)
target_link_libraries(traffic_bar_plugin ${GAZEBO_LIBRARIES})

install(TARGETS
  traffic_light_plugin
  traffic_bar_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
