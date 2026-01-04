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

#include "vfi_configuration_file_yaml.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <iomanip>

namespace DQ_robotics_extensions
{

class VFIConfigurationFileYaml::Impl
{
public:
    YAML::Node config_;
    std::string config_file_;
    Impl()
    {

    };


    /**
     * @brief get_vector_list returns a string vector containing the data from a given YAML node.
     * @param node A YAML node
     * @param key_name The key name to display an error message.
     * @return The desired string vector.
     */
    std::vector<std::string> get_vector_list(const YAML::Node& node, const std::string& key_name)
    {
        std::vector<std::string> entities;
        if (node.IsSequence()) {
            entities = node.as<std::vector<std::string>>();
            if (entities.empty())
                throw std::runtime_error(key_name + "is an empty list!");
        }
        return entities;
    }

    /**
     * @brief join_vector create a string from a string vector.
     * @param vec The string vector
     * @param delimiter A string to separate the elements of the vector
     * @return The desired string
     */
    std::string join_vector(const std::vector<std::string>& vec, const std::string& delimiter = ", ") {
        std::string result;
        for (size_t i = 0; i < vec.size(); ++i) {
            result += vec[i];
            if (i < vec.size() - 1) {
                result += delimiter;
            }
        }
        return result;
    }

    /**
     * @brief log_complete_raw_data displays on the terminal the raw data vector.
     * @param raw_data The raw data vector obtained from the YAML file.
     */
    void log_complete_raw_data(const std::vector<DQ_robotics_extensions::VFIConfigurationFile::RawData>& raw_data) {
        std::cout << "╔══════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║       VFI Configuration File Parser - C++17         ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;
        std::cout << "\n==========================================" << std::endl;
        std::cout << "RAW DATA LOG (" << raw_data.size() << " items)" << std::endl;
        std::cout << "==========================================" << std::endl;

        for (size_t i = 0; i < raw_data.size(); ++i) {
            std::cout << "\n\n[" << i + 1 << "/" << raw_data.size() << "] ";

            if (std::holds_alternative<DQ_robotics_extensions::VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_RAW_DATA>(raw_data[i])) {
                std::cout << "ENVIRONMENT_TO_ROBOT" << std::endl;
                std::cout << std::string(50, '-') << std::endl;

                auto& env_data = std::get<DQ_robotics_extensions::VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_RAW_DATA>(raw_data[i]);

                std::cout << std::left << std::setw(35) << "  vfi_type:" << env_data.vfi_type << std::endl;
                std::cout << std::setw(35) << "  cs_entity_environment:"
                          << "[" << join_vector(env_data.cs_entity_environment) << "]" << std::endl;
                std::cout << std::setw(35) << "  cs_entity_robot:"
                          << "[" << join_vector(env_data.cs_entity_robot) << "]" << std::endl;
                std::cout << std::setw(35) << "  entity_environment_primitive_type:"
                          << env_data.entity_environment_primitive_type << std::endl;
                std::cout << std::setw(35) << "  entity_robot_primitive_type:"
                          << env_data.entity_robot_primitive_type << std::endl;
                std::cout << std::setw(35) << "  robot_index:" << env_data.robot_index << std::endl;
                std::cout << std::setw(35) << "  joint_index:" << env_data.joint_index << std::endl;
                std::cout << std::setw(35) << "  safe_distance:" << env_data.safe_distance << std::endl;
                std::cout << std::setw(35) << "  vfi_gain:" << env_data.vfi_gain << std::endl;
                std::cout << std::setw(35) << "  direction:" << env_data.direction << std::endl;
                std::cout << std::setw(35) << "  tag:" << env_data.tag << std::endl;

            } else {
                std::cout << "ROBOT_TO_ROBOT" << std::endl;
                std::cout << std::string(50, '-') << std::endl;

                auto& robot_data = std::get<DQ_robotics_extensions::VFIConfigurationFile::ROBOT_TO_ROBOT_RAW_DATA>(raw_data[i]);

                std::cout << std::left << std::setw(35) << "  vfi_type:" << robot_data.vfi_type << std::endl;
                std::cout << std::setw(35) << "  cs_entity_one:"
                          << "[" << join_vector(robot_data.cs_entity_one) << "]" << std::endl;
                std::cout << std::setw(35) << "  cs_entity_two:"
                          << "[" << join_vector(robot_data.cs_entity_two) << "]" << std::endl;
                std::cout << std::setw(35) << "  entity_one_primitive_type:"
                          << robot_data.entity_one_primitive_type << std::endl;
                std::cout << std::setw(35) << "  entity_two_primitive_type:"
                          << robot_data.entity_two_primitive_type << std::endl;
                std::cout << std::setw(35) << "  robot_index_one:" << robot_data.robot_index_one << std::endl;
                std::cout << std::setw(35) << "  robot_index_two:" << robot_data.robot_index_two << std::endl;
                std::cout << std::setw(35) << "  joint_index_one:" << robot_data.joint_index_one << std::endl;
                std::cout << std::setw(35) << "  joint_index_two:" << robot_data.joint_index_two << std::endl;
                std::cout << std::setw(35) << "  safe_distance:" << robot_data.safe_distance << std::endl;
                std::cout << std::setw(35) << "  vfi_gain:" << robot_data.vfi_gain << std::endl;
                std::cout << std::setw(35) << "  direction:" << robot_data.direction << std::endl;
                std::cout << std::setw(35) << "  tag:" << robot_data.tag << std::endl;
            }
        }
        std::cout << "\n==========================================" << std::endl;
        std::cout << "END OF LOG" << std::endl;
        std::cout << "==========================================" << std::endl;
    }

};

/**
 * @brief VFIConfigurationFileYaml::VFIConfigurationFileYaml ctor of the class.
 * @param config_file The configuration YAML file. This path must contain the file and its format.
 *                    Example: "/path_to_the_file/config_file.yaml"
 */
VFIConfigurationFileYaml::VFIConfigurationFileYaml(const std::string& config_file)
{
    impl_ = std::make_shared<VFIConfigurationFileYaml::Impl>();
    impl_->config_file_ = config_file;
    _extract_yaml_data();
}

/**
 * @brief VFIConfigurationFileYaml::_extract_yaml_data reads the YAML file and store the data on a RAW_DATA vector.
 */
void VFIConfigurationFileYaml::_extract_yaml_data()
{
    try {
        impl_->config_ = YAML::LoadFile(impl_->config_file_);
        const YAML::Node& vfi_file_version = impl_->config_["vfi_file_version"]; //Aliasing
        const YAML::Node& zero_indexed = impl_->config_["zero_indexed"]; //Aliasing
        const YAML::Node& vfi_array = impl_->config_["vfi_array"]; //Aliasing

        for (const auto& parameter : vfi_array) {
            try {
                std::string vfi_type = parameter["vfi_type"].as<std::string>();

                if (vfi_type == "ENVIRONMENT_TO_ROBOT") {
                    ENVIRONMENT_TO_ROBOT_RAW_DATA env_data;
                    env_data.vfi_type = vfi_type;
                    env_data.cs_entity_environment  = impl_->get_vector_list(parameter["cs_entity_environment"],
                                                                                       "cs_entity_environment");
                    env_data.cs_entity_robot  = impl_->get_vector_list(parameter["cs_entity_robot"],
                                                                                 "cs_entity_robot");
                    env_data.entity_environment_primitive_type = parameter["entity_environment_primitive_type"].as<std::string>();
                    env_data.entity_robot_primitive_type = parameter["entity_robot_primitive_type"].as<std::string>();
                    env_data.robot_index = parameter["robot_index"].as<int>();
                    env_data.joint_index = parameter["joint_index"].as<int>();
                    env_data.safe_distance = parameter["safe_distance"].as<double>();
                    env_data.vfi_gain = parameter["vfi_gain"].as<double>();
                    env_data.direction = parameter["direction"].as<std::string>();
                    env_data.tag = parameter["tag"].as<std::string>();
                    raw_data_.push_back(env_data);

                }else if (vfi_type == "ROBOT_TO_ROBOT") {
                    ROBOT_TO_ROBOT_RAW_DATA robot_data;
                    robot_data.vfi_type = vfi_type;
                    robot_data.cs_entity_one  = impl_->get_vector_list(parameter["cs_entity_one"],
                                                                                 "cs_entity_one");
                    robot_data.cs_entity_two = impl_->get_vector_list(parameter["cs_entity_two"],
                                                                                "cs_entity_robot");
                    robot_data.entity_one_primitive_type = parameter["entity_one_primitive_type"].as<std::string>();
                    robot_data.entity_two_primitive_type = parameter["entity_two_primitive_type"].as<std::string>();
                    robot_data.robot_index_one = parameter["robot_index_one"].as<int>();
                    robot_data.robot_index_two = parameter["robot_index_two"].as<int>();
                    robot_data.joint_index_one = parameter["joint_index_one"].as<int>();
                    robot_data.joint_index_two = parameter["joint_index_two"].as<int>();
                    robot_data.safe_distance = parameter["safe_distance"].as<double>();
                    robot_data.vfi_gain = parameter["vfi_gain"].as<double>();
                    robot_data.direction = parameter["direction"].as<std::string>();
                    robot_data.tag = parameter["tag"].as<std::string>();
                    raw_data_.push_back(robot_data);

                }else {
                    throw std::runtime_error("Unknown VFI type: " + vfi_type);
                }
            }
            catch (const YAML::Exception& e) {
            std::cerr << "Error parsing VFI item: " << e.what() << std::endl;
            }
        }
    }
    catch(const YAML::BadFile& e)
    {
        std::cerr << e.msg << std::endl;
        throw std::runtime_error(e.msg);
    }
    catch(const YAML::ParserException& e)
    {
        std::cerr << e.msg << std::endl;
        throw std::runtime_error(e.msg);
    }

}


/**
 * @brief VFIConfigurationFileYaml::get_raw_data gets the raw data vector from a YAML file.
 * @return A raw data vector.
 */
std::vector<VFIConfigurationFile::RawData> VFIConfigurationFileYaml::get_raw_data()
{
    return raw_data_;
}

/**
 * @brief VFIConfigurationFileYaml::show_raw_data displays on the terminal the raw data vector.
 * @param raw_data The raw data vector obtained from the YAML file.
 */
void VFIConfigurationFileYaml::show_raw_data(const std::vector<RawData>& raw_data)
{
    impl_->log_complete_raw_data(raw_data);
}

}
