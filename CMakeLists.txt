cmake_minimum_required(VERSION 3.8)
project(vps_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge 4.1.0 REQUIRED)
find_package(OpenCV 5  REQUIRED)
find_package(CURL REQUIRED)

# add_library(${PROJECT_NAME} SHARED
#   src/match_streetview.cpp
# )

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}

)

add_executable(${PROJECT_NAME}
  src/match_streetview.cpp
  src/vps_slam.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CURL_LIBRARIES}
)
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

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
