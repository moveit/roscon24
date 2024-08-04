
cmake_minimum_required(VERSION 3.18)
project(exercise1-1)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
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
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hot_dog
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2_geometry_msgs
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(
    ${PROJECT_NAME} SHARED
    src/hot_dog_scenario.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}
  PRIVATE
  cxx_std_17
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_features(${PROJECT_NAME}_node
  PRIVATE
  cxx_std_17
)

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}  ${PROJECT_NAME}_node
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
