cmake_minimum_required(VERSION 3.8)
project(hik_camera)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# enforcing cleaner code.
add_definitions(-Wall -Werror)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(camera_node REQUIRED)

# 创建HIK相机驱动库（作为pluginlib插件）
add_library(hik_camera_driver SHARED
  src/hik_camera_driver.cpp
)

target_include_directories(hik_camera_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  hikSDK/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(hik_camera_driver ${OpenCV_LIBS})

ament_target_dependencies(hik_camera_driver
  pluginlib
  camera_node
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(hik_camera_driver PUBLIC hikSDK/lib/amd64)
  install(DIRECTORY hikSDK/lib/amd64/ DESTINATION lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(hik_camera_driver PUBLIC hikSDK/lib/arm64)
  install(DIRECTORY hikSDK/lib/arm64/ DESTINATION lib)
else()
  message(
    FATAL_ERROR
      "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(hik_camera_driver FormatConversion MediaProcess
                      MvCameraControl MVRender MvUsb3vTL)

# 安装库
install(TARGETS hik_camera_driver
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
