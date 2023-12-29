cmake_minimum_required(VERSION 3.5)

project(kr_gcs_ui LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grp_control_msg REQUIRED)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${PROJECT_SOURCE_DIR}/ui)

file (GLOB ${PROJECT_NAME}_SRCS
  ui/*.ui
  src/*.cpp
  include/*.hpp
  asset/*/*.qrc
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
ament_target_dependencies(${PROJECT_NAME} rclcpp grp_control_msg)
target_link_libraries(${PROJECT_NAME}
  Qt${QT_VERSION_MAJOR}::Widgets
  modbus
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
