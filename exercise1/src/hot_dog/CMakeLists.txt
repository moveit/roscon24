
cmake_minimum_required(VERSION 3.18)
project(hot_dog)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#############
## Install ##
#############

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../assets/hotdog
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../assets/mustard-bottle
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
