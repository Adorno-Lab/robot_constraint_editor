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

#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <filesystem>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <map>
#include <fstream>




namespace DQ_robotics_extensions
{

// Explicit instantiations for all expected types
template void RobotConstraintEditor::edit_data<int>(const std::string&, const std::string&, const int&);
template void RobotConstraintEditor::edit_data<double>(const std::string&, const std::string&, const double&);
template void RobotConstraintEditor::edit_data<std::string>(const std::string&, const std::string&, const std::string&);
template void RobotConstraintEditor::edit_data<std::vector<std::string>>(const std::string&, const std::string&, const std::vector<std::string>&);



class RobotConstraintEditor::Impl
{
public:
    YAML::Node config_;
    const int vfi_file_version_ = 2; // default value
    bool zero_indexed_ = true; // default value
    std::shared_ptr<VFIConfigurationFileYaml> vfi_config_file_yaml_;

    std::map<std::string, VFIConfigurationFile::RawData> yaml_raw_data_map_;

    /**
     * @brief _is_the_same_type checks if two RawData structures have the same type.
     * @param data1
     * @param data2
     * @return True if data1 and data2 have the same type. False otherwise.
     */
    bool _is_the_same_type(const VFIConfigurationFile::RawData& data1, const VFIConfigurationFile::RawData& data2)
    {
        return data1.index() == data2.index();
    }


    /**
     * @brief _load_data_from_yaml_file load data from a YAML file.
     * @param config_file The path to the YAML file, including its name and extension
     * @return The raw data vector.
     */
    std::vector<VFIConfigurationFile::RawData> _load_data_from_yaml_file(const std::string& config_file)
    {
        if (!vfi_config_file_yaml_)
            vfi_config_file_yaml_ = std::make_shared<VFIConfigurationFileYaml>(config_file);
        return vfi_config_file_yaml_->get_raw_data();
    }

    /**
     * @brief _extract_tag This method gets the tag of a RawData element.
     * @param raw_data
     * @return The desired tag
     */
    std::string _extract_tag(const VFIConfigurationFile::RawData& raw_data) {
        return std::visit([](auto&& arg) -> std::string {
            return arg.tag;
        }, raw_data);
    }

    /**
     * @brief is_tag_in_map checks if a tag is in the map
     * @param tag The tag to check
     * @return True if the tag is on the map. False otherwise.
     */
    bool is_tag_in_map(const std::string& tag)
    {
        return (yaml_raw_data_map_.find(tag) == yaml_raw_data_map_.end()) ? false : true;
    }

    Impl()
    {

    };
};

/**
 * @brief RobotConstraintEditor::RobotConstraintEditor ctor of the class
 */
RobotConstraintEditor::RobotConstraintEditor() {
    impl_ = std::make_shared<RobotConstraintEditor::Impl>();
}


/**
 * @brief RobotConstraintEditor::load_data
 * @param config_file The path to the YAML file, including its name and extension.
 */
void  RobotConstraintEditor::load_data(const std::string& config_file)
{
    std::vector<VFIConfigurationFile::RawData> raw_data = impl_->_load_data_from_yaml_file(config_file);
    impl_->yaml_raw_data_map_.clear();
    for (const auto& data : raw_data) {
        std::string tag = impl_->_extract_tag(data);
        impl_->yaml_raw_data_map_.try_emplace(tag, data);
    }
}

/**
 * @brief RobotConstraintEditor::replace_data removes the data stored in the corresponding tag, and adds
 *              the new data. The new data will will be tagged automatically.
 * @param tag The tag of the data to be removed
 * @param data The new data to add.
 */
void RobotConstraintEditor::replace_data(const std::string& tag, const VFIConfigurationFile::RawData& data)
{
    if (impl_->vfi_config_file_yaml_)
    {
        try{
            remove_data(tag);
            add_data(data);
        } catch (const std::runtime_error& e) {
            std::cerr<<e.what()<<std::endl;
            throw std::runtime_error("RobotConstraintEditor::edit_data: Fail to update the VFI data!");
        }

    }else
        throw std::runtime_error("There is no any data to replace!");
}

/**
 * @brief RobotConstraintEditor::add_data adds data to compose the YAML file.
 * @param data
 */
void RobotConstraintEditor::add_data(const VFIConfigurationFile::RawData& data)
{
    const std::string tag = impl_->_extract_tag(data);
    if (impl_->is_tag_in_map(tag))
        throw std::runtime_error("Tag '" + tag + "' is being used!");
    impl_->yaml_raw_data_map_.try_emplace(tag, data);
}

/**
 * @brief RobotConstraintEditor::remove_data removes data
 * @param tag
 */
void RobotConstraintEditor::remove_data(const std::string& tag)
{
    if (!impl_->is_tag_in_map(tag))
        throw std::runtime_error("Tag '" + tag + "' not found!");
    impl_->yaml_raw_data_map_.erase(tag);
}

/**
 * @brief RobotConstraintEditor::edit_data modifies the value of a key in the specified tagged data.
 * @param tag The tag that identifies the data to be edited.
 * @param key The key you want to modify.
 * @param value The new value of the key.
 */
template<typename T>
void RobotConstraintEditor::edit_data(const std::string& tag, const std::string& key, const T& value)
{
    // Check if tag exists
    if (!impl_->is_tag_in_map(tag))
        throw std::runtime_error("Tag '" + tag + "' not found!");

    auto& raw_data = impl_->yaml_raw_data_map_.at(tag);
    bool modified = false;

    std::visit([&](auto&& arg) {
        using DataType = std::decay_t<decltype(arg)>;

        // Helper function to assign value with type checking
        auto assign_if_match = [&](auto& field, const std::string& field_name) -> bool {
            if (key != field_name) return false;

            using FieldType = std::decay_t<decltype(field)>;

            // Check if types are compatible
            if constexpr (std::is_same_v<FieldType, T>) {
                field = value;
                return true;
            } else if constexpr (std::is_convertible_v<T, FieldType>) {
                field = value;  // Allow implicit conversions (int to double, etc.)
                return true;
            } else {
                throw std::runtime_error("Type mismatch for field '" + key +
                                         "'. Expected: " + typeid(FieldType).name() +
                                         ", Got: " + typeid(T).name());
            }
        };

        if constexpr (std::is_same_v<DataType, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_RAW_DATA>) {
            // String fields
            if (assign_if_match(arg.vfi_type, "vfi_type")) modified = true;
            else if (assign_if_match(arg.entity_environment_primitive_type, "entity_environment_primitive_type")) modified = true;
            else if (assign_if_match(arg.entity_robot_primitive_type, "entity_robot_primitive_type")) modified = true;
            else if (assign_if_match(arg.direction, "direction")) modified = true;

            // Integer fields
            else if (assign_if_match(arg.robot_index, "robot_index")) modified = true;
            else if (assign_if_match(arg.joint_index, "joint_index")) modified = true;

            // Double fields (also accept int via conversion)
            else if (assign_if_match(arg.safe_distance, "safe_distance")) modified = true;
            else if (assign_if_match(arg.vfi_gain, "vfi_gain")) modified = true;

            // Vector fields
            else if (assign_if_match(arg.cs_entity_environment, "cs_entity_environment")) modified = true;
            else if (assign_if_match(arg.cs_entity_robot, "cs_entity_robot")) modified = true;

            // Special handling for tag - update map key
            else if (key == "tag") {
                if constexpr (std::is_same_v<T, std::string> ||
                              std::is_convertible_v<T, std::string>) {
                    std::string old_tag = arg.tag;
                    arg.tag = value;

                    // Update the map key
                    auto node_handler = impl_->yaml_raw_data_map_.extract(old_tag);
                    if (!node_handler.empty()) {
                        node_handler.key() = value;
                        impl_->yaml_raw_data_map_.insert(std::move(node_handler));
                    }
                    modified = true;
                } else {
                    throw std::runtime_error("Tag must be convertible to string");
                }
            }
            else {
                throw std::runtime_error("Key '" + key + "' not found for ENVIRONMENT_TO_ROBOT");
            }

        } else if constexpr (std::is_same_v<DataType, VFIConfigurationFile::ROBOT_TO_ROBOT_RAW_DATA>) {
            // String fields
            if (assign_if_match(arg.vfi_type, "vfi_type")) modified = true;
            else if (assign_if_match(arg.entity_one_primitive_type, "entity_one_primitive_type")) modified = true;
            else if (assign_if_match(arg.entity_two_primitive_type, "entity_two_primitive_type")) modified = true;
            else if (assign_if_match(arg.direction, "direction")) modified = true;

            // Integer fields
            else if (assign_if_match(arg.robot_index_one, "robot_index_one")) modified = true;
            else if (assign_if_match(arg.robot_index_two, "robot_index_two")) modified = true;
            else if (assign_if_match(arg.joint_index_one, "joint_index_one")) modified = true;
            else if (assign_if_match(arg.joint_index_two, "joint_index_two")) modified = true;

            // Double fields
            else if (assign_if_match(arg.safe_distance, "safe_distance")) modified = true;
            else if (assign_if_match(arg.vfi_gain, "vfi_gain")) modified = true;

            // Vector fields
            else if (assign_if_match(arg.cs_entity_one, "cs_entity_one")) modified = true;
            else if (assign_if_match(arg.cs_entity_two, "cs_entity_two")) modified = true;

            // Special handling for tag
            // IF the tag is modified, we need to update the new tag in the map.
            else if (key == "tag") {
                if constexpr (std::is_same_v<T, std::string> ||
                              std::is_convertible_v<T, std::string>) {
                    std::string old_tag = arg.tag;
                    arg.tag = value;

                    // Update the map key
                    auto node_handler = impl_->yaml_raw_data_map_.extract(old_tag);
                    if (!node_handler.empty()) {
                        node_handler.key() = value;
                        impl_->yaml_raw_data_map_.insert(std::move(node_handler));
                    }
                    modified = true;
                } else {
                    throw std::runtime_error("Tag must be convertible to string");
                }
            }
            else {
                throw std::runtime_error("Key '" + key + "' not found for ROBOT_TO_ROBOT");
            }
        }
    }, raw_data);

    if (!modified) {
        throw std::runtime_error("Failed to edit field '" + key + "' for tag '" + tag + "'");
    }


}


/**
 * @brief RobotConstraintEditor::save_data saves the current data in a YAML file.
 * @param path_config_file The path to the YAML file, including its name and format.
 * @param vfi_file_version The version you want to specify.
 * @param zero_indexed The desired zero indexed flag you want to specify.
 */
void RobotConstraintEditor::save_data(const std::string& path_config_file,
                                      const int &vfi_file_version,
                                      const bool &zero_indexed)
{
    try {
        if (path_config_file.empty())
            throw std::runtime_error("path_config_file path cannot be empty!");

        // Create directory if it doesn't exist
        std::filesystem::path file_path(path_config_file);
        std::filesystem::path directory = file_path.parent_path();

        if (!directory.empty() && !std::filesystem::exists(directory)) {
            std::cout << "Creating directory: " << directory << std::endl;
            std::filesystem::create_directories(directory);
        }

        std::ofstream file(path_config_file);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + path_config_file);
        }

        // Write header
        file << "vfi_file_version: " << vfi_file_version << "\n";
        file << "zero_indexed: " << (zero_indexed ? "true" : "false") << "\n";
        file << "vfi_array:\n";

        for (const auto& [tag, raw_data] : impl_->yaml_raw_data_map_) {

            file << "  -\n";
            std::visit([&file](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_RAW_DATA>) {
                    file << "    vfi_type: \"" << arg.vfi_type << "\"\n";


                    file << "    cs_entity_environment: [";
                    for (size_t i = 0; i < arg.cs_entity_environment.size(); ++i) {
                        file << "\"" << arg.cs_entity_environment[i] << "\"";
                        if (i < arg.cs_entity_environment.size() - 1) file << ", ";
                    }
                    file << "]\n";


                    file << "    cs_entity_robot: [";
                    for (size_t i = 0; i < arg.cs_entity_robot.size(); ++i) {
                        file << "\"" << arg.cs_entity_robot[i] << "\"";
                        if (i < arg.cs_entity_robot.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    file << "    entity_environment_primitive_type: \""
                         << arg.entity_environment_primitive_type << "\"\n";
                    file << "    entity_robot_primitive_type: \""
                         << arg.entity_robot_primitive_type << "\"\n";
                    file << "    robot_index: " << arg.robot_index << "\n";
                    file << "    joint_index: " << arg.joint_index << "\n";
                    file << "    safe_distance: " << arg.safe_distance << "\n";

                    // vfi_gain with .0 for integers
                    file << "    vfi_gain: ";
                    if (arg.vfi_gain == static_cast<int>(arg.vfi_gain)) {
                        file << arg.vfi_gain << ".0";
                    } else {
                        file << arg.vfi_gain;
                    }
                    file << "\n";

                    file << "    direction: \"" << arg.direction << "\"\n";
                    file << "    tag: \"" << arg.tag << "\"\n";

                } else if constexpr (std::is_same_v<T, VFIConfigurationFile::ROBOT_TO_ROBOT_RAW_DATA>) {
                    file << "    vfi_type: \"" << arg.vfi_type << "\"\n";

                    // cs_entity_one
                    file << "    cs_entity_one: [";
                    for (size_t i = 0; i < arg.cs_entity_one.size(); ++i) {
                        file << "\"" << arg.cs_entity_one[i] << "\"";
                        if (i < arg.cs_entity_one.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    // cs_entity_two
                    file << "    cs_entity_two: [";
                    for (size_t i = 0; i < arg.cs_entity_two.size(); ++i) {
                        file << "\"" << arg.cs_entity_two[i] << "\"";
                        if (i < arg.cs_entity_two.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    file << "    entity_one_primitive_type: \""
                         << arg.entity_one_primitive_type << "\"\n";
                    file << "    entity_two_primitive_type: \""
                         << arg.entity_two_primitive_type << "\"\n";
                    file << "    robot_index_one: " << arg.robot_index_one << "\n";
                    file << "    robot_index_two: " << arg.robot_index_two << "\n";
                    file << "    joint_index_one: " << arg.joint_index_one << "\n";
                    file << "    joint_index_two: " << arg.joint_index_two << "\n";
                    file << "    safe_distance: " << arg.safe_distance << "\n";

                    // vfi_gain with .0 for integers
                    file << "    vfi_gain: ";
                    if (arg.vfi_gain == static_cast<int>(arg.vfi_gain)) {
                        file << arg.vfi_gain << ".0";
                    } else {
                        file << arg.vfi_gain;
                    }
                    file << "\n";

                    file << "    direction: \"" << arg.direction << "\"\n";
                    file << "    tag: \"" << arg.tag << "\"\n";
                }
            }, raw_data);
        }

        file.close();

        std::cout << "Successfully saved " << impl_->yaml_raw_data_map_.size()
                  << " VFI entries to: " << path_config_file << std::endl;

    } catch (const std::filesystem::filesystem_error& e) {
        throw std::runtime_error("Filesystem error: " + std::string(e.what()));
    } catch (const std::exception& e) {
        throw std::runtime_error("Error in save_data: " + std::string(e.what()));
    }
}

/**
 * @brief RobotConstraintEditor::get_raw_data returns the raw data vector
 * @return The desired vector
 */
std::vector<VFIConfigurationFile::RawData> RobotConstraintEditor::get_raw_data()
{
    std::vector<VFIConfigurationFile::RawData> raw_data;
    raw_data.reserve(impl_->yaml_raw_data_map_.size());
    for (auto& pair : impl_->yaml_raw_data_map_)
        raw_data.push_back(pair.second);
    return raw_data;
}

}
