cmake_minimum_required(VERSION 3.8)
project(dakae_interfaces)

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

# 1. 필요한 빌드 도구들 찾기
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 인터페이스 생성 (가장 핵심)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubeInfo.msg"
  "msg/MarkerInfo.msg"
  "msg/TaskInfo.msg"
  "msg/ObjectInfo.msg"
  "srv/DetectCubes.srv"
  "srv/DetectMarkers.srv"
  "srv/DetectObjects.srv"
  "srv/ExecuteTasks.srv"
  "srv/MoveGripper.srv"
  "action/PickAndPlace.action"
  DEPENDENCIES geometry_msgs
)
ament_package()
