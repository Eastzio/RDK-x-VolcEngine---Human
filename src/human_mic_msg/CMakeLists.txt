cmake_minimum_required(VERSION 3.8)
project(human_mic_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PcmMsg.msg"
  "srv/GetDeviceType.srv"
  "srv/GetOfflineResult.srv"
  "srv/SetAwakeWord.srv"
  "srv/SwitchMic.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 先导出依赖
ament_export_dependencies(rosidl_default_runtime)

# 然后只调用一次ament_package()，并确保它是文件的最后一行
ament_package()
