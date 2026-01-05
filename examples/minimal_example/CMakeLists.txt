cmake_minimum_required(VERSION 3.5...3.26)

project(examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

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


include_directories(${PROJECT_NAME}  include)



include_directories(../../include)
add_library(vfi_config_yaml
           ../../src/dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.cpp
           ../../src/dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.cpp
           ../../src/dqrobotics_extensions/robot_constraint_editor/utils.cpp
)
target_link_libraries(vfi_config_yaml
           yaml-cpp::yaml-cpp
)

add_executable(${PROJECT_NAME} main.cpp
           )

target_link_libraries(${PROJECT_NAME}
           vfi_config_yaml
           yaml-cpp::yaml-cpp
)

# COPY the Yaml config file to the build folder
set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../design/specs_document/config_file.yaml)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${YAML_FILE}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)

