cmake_minimum_required(VERSION 3.8)
project(human_mic_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# 根据架构选择库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    link_directories(lib/arm64)
else()
    link_directories(lib/x64)
endif()

# 安装资源文件
install(DIRECTORY launch config feedback_voice audio
  DESTINATION share/${PROJECT_NAME})

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(human_mic_msg REQUIRED)
find_package(jsoncpp REQUIRED)

# 恢复原有的添加ros2_service.cpp
# 将 human_mic 改为 human_mic
add_executable(human_mic src/human_mic.cpp src/ros2_service.cpp)
ament_target_dependencies(human_mic 
  rclcpp 
  std_msgs 
  human_mic_msg 
  serial
)
target_link_libraries(human_mic 
  -ljsoncpp
)

# 恢复原有的库链接
add_executable(voice_control src/voice_control.cpp)
ament_target_dependencies(voice_control 
  rclcpp 
  std_msgs 
  human_mic_msg
)
target_link_libraries(voice_control 
  offline_record_lib rt dl pthread stdc++ asound
  jsoncpp
)

add_executable(call_recognition src/call_recognition.cpp)
ament_target_dependencies(call_recognition 
  rclcpp 
  std_msgs 
  human_mic_msg
)
target_link_libraries(call_recognition 
  msc rt dl pthread asound
  jsoncpp
)

# 安装可执行文件
install(TARGETS
  voice_control
  call_recognition
  human_mic
  DESTINATION lib/${PROJECT_NAME})

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
