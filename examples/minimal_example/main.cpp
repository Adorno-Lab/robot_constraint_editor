#include "vfi_configuration_file_yaml.hpp"
#include <iostream>


using namespace DQ_robotics_extensions;



int main()
{

    auto ri = VFIConfigurationFileYaml("config_file.yaml");
    std::vector<VFIConfigurationFile::RawData> raw_data = ri.get_raw_data();



    ri.show_raw_data(raw_data);



    return 0;
}
