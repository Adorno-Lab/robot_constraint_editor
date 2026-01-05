![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/juanjqo/capybara_toolkit?color=orange)![Static Badge](https://img.shields.io/badge/status-experimental-red)

# robot_constraint_editor
A graphical editor to create and manage configuration files for the [robot_constraint_manager](https://github.com/Adorno-Lab/robot_constraint_manager)


### Prerequisites 

#### [Yaml-cpp](https://github.com/jbeder/yaml-cpp)

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

```
brew update
brew install yaml-cpp
```
