cmake_minimum_required(VERSION 3.8)
project(huaray_camera)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# enforcing cleaner code.
add_definitions(-Wall -Werror)

find_library(
  CAMERA_LIB
  NAMES MVSDK
  PATHS ./huaraySDK/lib
  NO_DEFAULT_PATH
  REQUIRED)

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(camera_node REQUIRED)

# 创建华睿相机驱动库（作为pluginlib插件）
add_library(huaray_camera_driver SHARED
  src/huaray_camera_driver.cpp
)

target_include_directories(huaray_camera_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  huaraySDK/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(huaray_camera_driver ${OpenCV_LIBS} ${CAMERA_LIB} rt)

ament_target_dependencies(huaray_camera_driver
  pluginlib
  camera_node
)

message(STATUS "CAMERA_LIB: ${CAMERA_LIB}")

# 安装库
install(TARGETS huaray_camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 导出pluginlib插件
pluginlib_export_plugin_description_file(camera_node plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint
       ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
