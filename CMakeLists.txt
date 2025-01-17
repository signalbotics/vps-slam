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
find_package(OpenCV 5 REQUIRED)
find_package(CURL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# add_library(${PROJECT_NAME} SHARED
#   src/match_streetview.cpp
# )

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/vps_slam.cpp
  src/match_streetview.cpp
  src/pose_estimator.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  ${CURL_LIBRARIES}
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  params
  DESTINATION share/${PROJECT_NAME}
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
