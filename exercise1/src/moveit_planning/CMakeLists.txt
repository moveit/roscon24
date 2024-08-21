
cmake_minimum_required(VERSION 3.18)
project(moveit_planning)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometric_shapes
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  std_msgs
  rclcpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

# Example 1-1

add_executable(${PROJECT_NAME}1-1
  src/exercise1-1/main.cpp
)

target_include_directories(${PROJECT_NAME}1-1
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

target_compile_options(${PROJECT_NAME}1-1
  PRIVATE
  -Wall
  -Wextra
  -Wshadow
  -Wnon-virtual-dtor
  -pedantic

  -Wold-style-cast
  -Wcast-align
  -Wunused
  -Woverloaded-virtual
  -Wpedantic
  -Wconversion
  -Wsign-conversion
  -Wmisleading-indentation
  -Wduplicated-cond
  -Wduplicated-branches
  -Wlogical-op
  -Wnull-dereference
  -Wuseless-cast
  -Wdouble-promotion
  -Wformat=2
  -Wimplicit-fallthrough
)

target_compile_features(${PROJECT_NAME}1-1
  PRIVATE
  cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}1-1 ${THIS_PACKAGE_INCLUDE_DEPENDS})


#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}1-1
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../assets/hotdog
  DESTINATION share/${PROJECT_NAME})


ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
