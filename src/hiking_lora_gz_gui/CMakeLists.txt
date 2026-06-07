cmake_minimum_required(VERSION 3.16)
project(hiking_lora_gz_gui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(Qt5 REQUIRED COMPONENTS Core Quick Qml)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(HikingLoraKeyPlugin SHARED
  src/HikingLoraKeyPlugin.cc
  resource/HikingLoraKeyPlugin.qrc
)

target_compile_features(HikingLoraKeyPlugin PRIVATE cxx_std_17)
target_link_libraries(HikingLoraKeyPlugin
  gz-gui8::gz-gui8
  gz-plugin2::register
  Qt5::Core
  Qt5::Quick
  Qt5::Qml
)
ament_target_dependencies(HikingLoraKeyPlugin
  rclcpp
  std_msgs
)

install(TARGETS HikingLoraKeyPlugin
  LIBRARY DESTINATION lib
)

ament_package()
