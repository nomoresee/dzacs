cmake_minimum_required(VERSION 3.0.2)
project(voice_module)

# 1. 查找 catkin 包时，加入 serial 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  serial
)

# 2. 声明 catkin 运行时依赖，加入 serial
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(voice_module src/voice_module.cpp)
target_link_libraries(voice_module ${catkin_LIBRARIES})

install(TARGETS voice_module
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)