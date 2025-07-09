cmake_minimum_required(VERSION 3.8)
project(human_aiui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ollama_ros_msgs REQUIRED)
find_package(human_mic_msg REQUIRED)

# 添加curl和jsoncpp依赖
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include_directories( /usr/include/alsa
                     ${CMAKE_CURRENT_LIST_DIR}/include
                     ${CMAKE_CURRENT_LIST_DIR}/src/utils
                     ${CMAKE_CURRENT_LIST_DIR}/src/jsoncpp
                     ${JSONCPP_INCLUDE_DIRS})

install(DIRECTORY launch AIUI
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB_RECURSE SOURCE_FILE
        "src/*.cpp")

# 根据架构选择库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  link_directories(${CMAKE_CURRENT_LIST_DIR}/libs/arm64)
else()
  link_directories(${CMAKE_CURRENT_LIST_DIR}/libs/x64)
endif()

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

# 添加human_mic_msg到依赖项
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  std_msgs 
  ollama_ros_msgs
  human_mic_msg
)

# 添加curl和jsoncpp到链接库
target_link_libraries(${PROJECT_NAME} 
  aiui 
  asound 
  pthread 
  stdc++ 
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PRIVATE 
  ${CURL_INCLUDE_DIRS}
)

install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}    
)

ament_package()
