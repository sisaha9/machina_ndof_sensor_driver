cmake_minimum_required(VERSION 3.8)
project(machina_ndof_sensor_driver_service)

# Default to C99.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_LOCKFREE "Enable lock free data structures" OFF)

if(ENABLE_LOCKFREE)
  add_compile_definitions(ENABLE_LOCKFREE)
endif()

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

generate_parameter_library(
  ${PROJECT_NAME}_parameters
  resources/parameters.yaml
)

set(${PROJECT_NAME}_SRC
  src/machina_ndof_sensor_driver_service_node.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_parameters
  iceoryx_hoofs::iceoryx_hoofs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "machina::service::ndof_sensor_driver::MachinaNDofSensorDriverServiceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
  EXECUTOR StaticSingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
