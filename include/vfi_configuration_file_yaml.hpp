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
    ~VFIConfigurationFileYaml() = default;

    std::vector<RawData> get_raw_data() override;
    VFIConfigurationFileYaml(const std::string& config_file);

    void show_raw_data(const std::vector<RawData>& raw_data);

};
}

