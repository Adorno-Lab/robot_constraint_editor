
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>

using namespace DQ_robotics_extensions;



int main()
{
    auto ri = VFIConfigurationFileYaml("config_file.yaml");
    ri.show_raw_data(ri.get_raw_data());
    return 0;
}
