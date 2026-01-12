/*
#    Copyright (c) 2024-2026 Adorno-Lab
#
#    robot_constraint_editor is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    robot_constraint_editor is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with robot_constraint_editor.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

#pragma once
#include <memory>
#include <vector>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file.hpp>

namespace DQ_robotics_extensions
{
class VFIConfigurationFileYaml: public VFIConfigurationFile
{
private:
    class Impl;
    std::shared_ptr<Impl> impl_;
public:
    ~VFIConfigurationFileYaml() = default;
    explicit VFIConfigurationFileYaml();

    // Override from VFIConfigurationFile
    void load_data(const std::string& config_file) override;
    std::vector<VFIConfigurationFile::Data> get_data() const override;
    int get_vfi_file_version() const override;
    bool is_zero_indexed() const override;
    void save_data(const std::vector<Data>& data,
                   const int& vfi_file_version,
                   const bool& zero_indexed,
                   const std::string& config_file) override;

};
}

