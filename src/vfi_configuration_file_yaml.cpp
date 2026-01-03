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

VFIConfigurationFileYaml::VFIConfigurationFileYaml(const std::string& config_file)
{
    impl_ = std::make_shared<VFIConfigurationFileYaml::Impl>();
    impl_->config_file_ = config_file;
    _extract_yaml_data();
}

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
                //std::cout<<vfi_type<<std::endl;

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

VFIConfigurationFileYaml::~VFIConfigurationFileYaml()
{

}

std::vector<VFIConfigurationFile::RawData> VFIConfigurationFileYaml::get_raw_data()
{
    return raw_data_;
}

void VFIConfigurationFileYaml::show_raw_data(const std::vector<RawData>& raw_data)
{
    impl_->log_complete_raw_data(raw_data);
}

}
