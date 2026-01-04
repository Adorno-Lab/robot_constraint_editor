/*
#    Copyright (c) 2024-2026 Adorno-Lab
#
#    RobotConstraintEditor is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RobotConstraintEditor is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with RobotConstraintEditor.  If not, see <https://www.gnu.org/licenses/>.
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

class RobotConstraintEditor
{
private:
    class Impl;
    std::shared_ptr<Impl> impl_;

public:
    RobotConstraintEditor();

    void load_data_from_yaml_file(const std::string& config_file);
    void edit_data(const std::string& tag, const VFIConfigurationFile::RawData& data);
    std::vector<VFIConfigurationFile::RawData> get_raw_data();
};
}
