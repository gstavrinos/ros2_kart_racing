cmake_minimum_required(VERSION 3.8)
project(race_steward)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(race_steward_msgs REQUIRED)

link_directories()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(${PROJECT_NAME} SHARED
  src/race_steward.cpp
  src/racer.cpp
  src/track.cpp
  src/gate.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  gazebo_ros
  race_steward_msgs
)

target_link_libraries(${PROJECT_NAME} )
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()

install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

