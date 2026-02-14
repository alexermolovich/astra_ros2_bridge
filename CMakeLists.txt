cmake_minimum_required(VERSION 3.8)
project(astra_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Find ROS 2 dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grasp_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)




# --- Find Astra SDK ---
set(ASTRA_ROOT "/home/alex/workspaces/astra_sdk")
list(APPEND CMAKE_MODULE_PATH "${ASTRA_ROOT}/samples/cmake")
find_package(Astra REQUIRED)

# --- Include directories ---
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${ASTRA_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${grasp_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
    )

# --- Create executable ---
add_executable(astra_ros2_bridge src/astra_brdige.cpp)

# --- Link libraries ---
target_link_libraries(astra_ros2_bridge
  ${ASTRA_LIBRARIES}
  ${OpenCV_LIBS}
)

# --- Set ROS 2 dependencies ---
ament_target_dependencies(astra_ros2_bridge
  rclcpp
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  grasp_msgs 
  )

# --- Set compile features ---
target_compile_features(astra_ros2_bridge PUBLIC c_std_99 cxx_std_17)

# --- Install target ---
install(TARGETS astra_ros2_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# --- Optional testing/linting ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
