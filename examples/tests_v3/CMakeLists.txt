cmake_minimum_required(VERSION 3.5...3.26)

project(tests_v3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Eigen3 REQUIRED)


if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if (APPLE)
    INCLUDE_DIRECTORIES(
           /usr/local/include/
           /usr/local/include/eigen3
           # Most recent versions of brew install here
           /opt/homebrew/include/
           /opt/homebrew/include/eigen3
       )
   ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
   # The library is installed here when using the regular cmake ., make, sudo make install
   LINK_DIRECTORIES(
       /usr/local/lib/
       /opt/homebrew/lib
       )
endif()



# By default, the library is built from the local sources of this repository.
# Use -DUSE_INSTALLED_LIBRARY=ON to use the installed library (find_package) instead.
option(USE_INSTALLED_LIBRARY "Use the installed robot_constraint_editor library instead of the local sources" OFF)

if(USE_INSTALLED_LIBRARY)
    find_package(robot_constraint_editor REQUIRED)
else()
    # Defines the robot_constraint_editor::robot_constraint_editor target from the local sources
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. ${CMAKE_CURRENT_BINARY_DIR}/robot_constraint_editor)
endif()

add_executable(${PROJECT_NAME}
               ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
           robot_constraint_editor::robot_constraint_editor
)

# COPY the Yaml config files to the build folder
set(YAML_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../design/specs_document/config_file.yaml
               ${CMAKE_CURRENT_SOURCE_DIR}/../../design/specs_document/config_file_v3.yaml)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${YAML_FILES}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)

