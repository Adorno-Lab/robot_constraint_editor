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
    RobotConstraintEditor(const std::shared_ptr<VFIConfigurationFile>& interface);

    // The editor contains either version 2 data (default) or a version 3 document.
    // load_data() and set_document() define the version.
    void load_data(const std::string& config_file);
    int get_vfi_file_version() const;

    // Version 2
    void add_data(const std::vector<VFIConfigurationFile::Data>& vector_data);
    void add_data(const VFIConfigurationFile::Data& data);
    void replace_data(const std::string& tag, const VFIConfigurationFile::Data& data);
    void save_data(const std::string& path_config_file,
                   const int& vfi_file_version,
                   const bool& zero_indexed);
    std::vector<VFIConfigurationFile::Data> get_data();

    // Versions 2 and 3
    void remove_data(const std::string& tag);

    template<typename T>
    void edit_data(const std::string& tag, const std::string& key, const T& value);

    // Version 3
    void set_document(const VFIConfigurationFile::DOCUMENT_V3& document);
    VFIConfigurationFile::DOCUMENT_V3 get_document() const;
    void validate() const;
    void save_document(const std::string& path_config_file);

    void set_metadata(const VFIConfigurationFile::METADATA& metadata);
    void set_robot(const VFIConfigurationFile::ROBOT& robot);

    void add_data(const VFIConfigurationFile::DataV3& data);
    void replace_data(const std::string& tag, const VFIConfigurationFile::DataV3& data);

    void add_environment_entity(const VFIConfigurationFile::ENVIRONMENT_ENTITY& entity);
    void add_robot_entity(const VFIConfigurationFile::ROBOT_ENTITY& entity);
    void replace_environment_entity(const std::string& name, const VFIConfigurationFile::ENVIRONMENT_ENTITY& entity);
    void replace_robot_entity(const std::string& name, const VFIConfigurationFile::ROBOT_ENTITY& entity);
    void rename_entity(const std::string& name, const std::string& new_name);
    void remove_entity(const std::string& name);
};
}
