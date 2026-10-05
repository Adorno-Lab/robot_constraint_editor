![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/Adorno-Lab/robot_constraint_editor?color=orange)![Static Badge](https://img.shields.io/badge/status-experimental-red)[![Docs](https://img.shields.io/badge/docs-GitHub_Pages-green)](https://adorno-lab.github.io/robot_constraint_editor/)

# robot_constraint_editor
A graphical editor to create and manage configuration files for the [robot_constraint_manager](https://github.com/Adorno-Lab/robot_constraint_manager).

The library reads, edits, validates, and writes the YAML configuration files (versions 2 and 3).
See the [version 3 specification](design/specs_document/config_file_specification_v3.md).


```shell
git clone https://github.com/Adorno-Lab/robot_constraint_editor
cd robot_constraint_editor
```


# Install

> [!NOTE]
> Non-sudo privileges? Create a custom prefix folder (e.g. `~/opt`) to hold `lib/` and `include/` without needing root. See [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html) for background.

## Prerequisites

- Eigen3 — `sudo apt install libeigen3-dev` (macOS: `brew install eigen`)
- [DQ Robotics](https://dqrobotics.github.io) — installed system-wide (e.g. via their apt PPA).
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

  Ubuntu:
  ```shell
  cd ~/Downloads && git clone https://github.com/jbeder/yaml-cpp
  cd ~/Downloads/yaml-cpp
  mkdir -p build && cd build
  cmake -DYAML_BUILD_SHARED_LIBS=on ..
  make
  sudo make install
  ```
  macOS:
  ```shell
  brew update
  brew install yaml-cpp
  ```

If you're installing any of the above without sudo, install them to the same custom prefix `~/opt`, and see the non-sudo instructions for `robot_constraint_editor` itself.


## Sudo users

```shell

# 1. Configure: choose Release, and (optionally) where to install it.
#    Omit -DCMAKE_INSTALL_PREFIX to use the system default (/usr/local on Linux).
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 2. Build the shared library.
cmake --build build -j$(nproc)

# 3. Install headers, library, and the exported CMake package.
sudo cmake --install build
```

## Non-sudo users

```shell

# 1. Configure: choose Release, and install to your own prefix instead of a system path.
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/opt

# 2. Build the shared library.
cmake --build build -j$(nproc)

# 3. Install headers, library, and the exported CMake package. No sudo needed.
cmake --install build
```

> [!TIP]
> If you skipped exporting `CMAKE_PREFIX_PATH` (along with `LD_LIBRARY_PATH`,
> `LIBRARY_PATH`, and `CPATH`) in `~/.bashrc` (see [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html)),
> any project that later does `find_package(robot_constraint_editor)` needs to be told
> where to look, since `$HOME/opt` isn't a default search path:
>
> ```shell
> cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/opt
> ```


# Usage

```cmake
find_package(robot_constraint_editor REQUIRED)
target_link_libraries(${YOUR_LIBRARY} PRIVATE
     robot_constraint_editor::robot_constraint_editor)
```

```cpp
#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
```

`RobotConstraintEditor` loads, edits, validates, and saves configuration files:

```cpp
using namespace DQ_robotics_extensions;

auto rce = RobotConstraintEditor(std::make_shared<VFIConfigurationFileYaml>());
rce.load_data("config_file_v3.yaml");                 // Version 3 files are validated when loaded

rce.edit_data("C3", "safe_distance", 0.08);           // Edit a VFI
rce.rename_entity("rsphere", "tool_sphere");          // The VFIs that use the entity are updated

rce.save_document("config_file_v3_edited.yaml");      // Validated before the file is written
```

See [examples/minimal_example](https://github.com/Adorno-Lab/robot_constraint_editor/tree/main/examples/minimal_example) for complete examples of versions 2 and 3.
