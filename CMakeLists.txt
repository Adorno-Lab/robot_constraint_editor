cmake_minimum_required(VERSION 3.16)

project(robot_constraint_editor
    VERSION 0.1.0
    DESCRIPTION "Library to create, edit, and validate the YAML configuration files of the robot_constraint_manager"
    LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Project-wide settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

# Eigen3 is header-only; Eigen3::Eigen is the imported target its Config file provides.
find_package(Eigen3 REQUIRED)

# yaml-cpp ships a CMake package config that provides the yaml-cpp::yaml-cpp target.
find_package(yaml-cpp REQUIRED)

# DQ Robotics (https://dqrobotics.github.io) does not ship a CMake package config.
# find_library() and find_path() locate it, giving absolute paths that are embedded
# in the exported target.
find_library(DQROBOTICS_LIBRARY dqrobotics)
find_path(DQROBOTICS_INCLUDE_DIR dqrobotics/DQ.h)
if(NOT DQROBOTICS_LIBRARY OR NOT DQROBOTICS_INCLUDE_DIR)
    message(FATAL_ERROR
        "DQ Robotics not found. Install DQ Robotics "
        "(see https://dqrobotics.github.io) before configuring robot_constraint_editor."
    )
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
    src/dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.cpp
    src/dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.cpp
    src/dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.cpp
    src/dqrobotics_extensions/robot_constraint_editor/utils.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual>
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${DQROBOTICS_INCLUDE_DIR}
)

# The public headers include <dqrobotics/DQ.h>, which includes Eigen.
# yaml-cpp is used only in the implementation (pImpl).
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        ${DQROBOTICS_LIBRARY}
    PRIVATE
        yaml-cpp::yaml-cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ---------------------------------------------------------------------------
# Install: headers, library, and the exported CMake package
# ---------------------------------------------------------------------------
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# How to use
# find_package(robot_constraint_editor REQUIRED)
# target_link_libraries(my_target PRIVATE robot_constraint_editor::robot_constraint_editor)
