cmake_minimum_required(VERSION 3.5...3.26)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

project(robot_constraint_editor LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
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


include_directories(${PROJECT_NAME}  include)

add_library(${PROJECT_NAME} SHARED
    src/dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.cpp
    src/dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.cpp
    src/dqrobotics_extensions/robot_constraint_editor/utils.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
)

target_link_libraries(${PROJECT_NAME}
        yaml-cpp
)

SET_TARGET_PROPERTIES(${PROJECT_NAME}
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp"
    )

INSTALL(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics_extensions/robot_constraint_editor/"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


# Other Headers
INSTALL(FILES
    include/dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file.hpp
    include/dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp
    include/dqrobotics_extensions/robot_constraint_editor/utils.hpp
    DESTINATION "include/dqrobotics_extensions/robot_constraint_editor")
