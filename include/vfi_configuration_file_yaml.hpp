#pragma once
#include <memory>
#include <vector>
#include "vfi_configuration_file.hpp"

namespace DQ_robotics_extensions
{
class VFIConfigurationFileYaml: public VFIConfigurationFile
{
private:
    class Impl;
    std::shared_ptr<Impl> impl_;
    void _extract_yaml_data();
    std::vector<RawData> raw_data_;
public:
    ~VFIConfigurationFileYaml();

    std::vector<RawData> get_raw_data() override;
    VFIConfigurationFileYaml(const std::string& config_file);

    void show_raw_data(const std::vector<RawData>& raw_data);

};
}

