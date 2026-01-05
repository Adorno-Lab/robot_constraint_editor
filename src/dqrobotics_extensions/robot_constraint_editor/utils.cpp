#include <dqrobotics_extensions/robot_constraint_editor/utils.hpp>
#include <iomanip>
#include <iostream>

namespace DQ_robotics_extensions {

std::string bool2string(const bool& flag)
{
    return flag == true ? std::string("true") : std::string("false");
}


/**
 * @brief join_vector create a string from a string vector.
 * @param vec The string vector
 * @param delimiter A string to separate the elements of the vector
 * @return The desired string
 */
std::string join_vector(const std::vector<std::string>& vec, const std::string& delimiter) {
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
 * @param vfi_file_version
 * @param zero_indexed
 */
void VFIConfigurationFileRawData::show_data(const std::vector<DQ_robotics_extensions::VFIConfigurationFile::RawData>& raw_data,
                           const int& vfi_file_version,
                           const bool& zero_indexed)
{
    std::cout << "╔══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║       VFI Configuration File Parser - C++17         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\n==========================================" << std::endl;
    std::cout << "VFI_FILE_VERSION: " << vfi_file_version << std::endl;
    std::cout << "Zero Indexed: " + bool2string(zero_indexed) << std::endl;
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



}
