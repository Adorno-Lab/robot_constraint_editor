#pragma once


#include <string>
#include <vector>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file.hpp>

namespace DQ_robotics_extensions {

std::string bool2string(const bool& flag);
std::string join_vector(const std::vector<std::string>& vec, const std::string& delimiter = ", ");

namespace  VFIConfigurationFileData {
    void show_data(const std::vector<DQ_robotics_extensions::VFIConfigurationFile::Data>& data,
                       const int& vfi_file_version,
                       const bool& zero_indexed);
    }

}
