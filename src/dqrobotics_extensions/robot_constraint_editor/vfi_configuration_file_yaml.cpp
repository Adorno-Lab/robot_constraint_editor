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

#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <dqrobotics_extensions/robot_constraint_editor/utils.hpp>

namespace DQ_robotics_extensions
{

class VFIConfigurationFileYaml::Impl
{
public:
    YAML::Node config_;
    std::string config_file_;
    int vfi_file_version_ = 2; // default value
    bool zero_indexed_ = true; // default value
    std::vector<RawData> raw_data_;
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
     * @brief VFIConfigurationFileYaml::_extract_yaml_data reads the YAML file and store the data on a RAW_DATA vector.
     */
    void _extract_yaml_data()
    {
        try {
            config_ = YAML::LoadFile(config_file_);

            if (config_["vfi_file_version"])
                vfi_file_version_ = config_["vfi_file_version"].as<int>();
            else
                std::cerr << "Warning: vfi_file_version not found, using default: "
                          << vfi_file_version_ << std::endl;


            if (config_["zero_indexed"])
                zero_indexed_ = config_["zero_indexed"].as<bool>();
            else
                std::cerr << "Warning: zero_indexed not found, using default: " + bool2string(zero_indexed_)<< std::endl;



            const YAML::Node& vfi_array = config_["vfi_array"]; //Aliasing



            for (const auto& parameter : vfi_array) {
                try {
                    std::string vfi_type = parameter["vfi_type"].as<std::string>();

                    if (vfi_type == "ENVIRONMENT_TO_ROBOT") {
                        ENVIRONMENT_TO_ROBOT_RAW_DATA env_data;
                        env_data.vfi_type = vfi_type;
                        env_data.cs_entity_environment  = get_vector_list(parameter["cs_entity_environment"],
                                                                                "cs_entity_environment");
                        env_data.cs_entity_robot  = get_vector_list(parameter["cs_entity_robot"],
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
                        robot_data.cs_entity_one  = get_vector_list(parameter["cs_entity_one"],
                                                                          "cs_entity_one");
                        robot_data.cs_entity_two = get_vector_list(parameter["cs_entity_two"],
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
    impl_->_extract_yaml_data();
}




/**
 * @brief VFIConfigurationFileYaml::get_raw_data gets the raw data vector from a YAML file.
 * @return A raw data vector.
 */
std::vector<VFIConfigurationFile::RawData> VFIConfigurationFileYaml::get_raw_data()
{
    return impl_->raw_data_;
}


/**
 * @brief VFIConfigurationFileYaml::get_vfi_file_version gets the vfi_file_version data from
 *              the YAML file.
 * @return The desired data.
 */
int VFIConfigurationFileYaml::get_vfi_file_version()
{
    return impl_->vfi_file_version_;
}

/**
 * @brief VFIConfigurationFileYaml::get_zero_indexed_status gets the zero_indexed data from
 *          the YAML file.
 * @return The desired data.
 */
bool VFIConfigurationFileYaml::get_zero_indexed_status()
{
    return impl_->zero_indexed_;
}

}
