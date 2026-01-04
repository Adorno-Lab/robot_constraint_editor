#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace DQ_robotics_extensions
{

class RobotConstraintEditor::Impl
{
public:
    enum class VFI_TYPE{
        ENVIRONMENT_TO_ROBOT,
        ROBOT_TO_ROBOT
    };
public:
    YAML::Node config_;
    //Sstd::string config_file_;
    int vfi_file_version_ = 2; // default value
    bool zero_indexed_ = true; // default value
    std::shared_ptr<VFIConfigurationFileYaml> vfi_config_file_yaml_;

    std::unordered_map<std::string, VFIConfigurationFile::RawData> yaml_raw_data_map_;

    VFI_TYPE _get_vfi_type(const VFIConfigurationFile::RawData& data)
    {
        if (std::holds_alternative<VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_RAW_DATA>(data))
            return VFI_TYPE::ENVIRONMENT_TO_ROBOT;
        else if (std::holds_alternative<VFIConfigurationFile::ROBOT_TO_ROBOT_RAW_DATA>(data))
            return  VFI_TYPE::ROBOT_TO_ROBOT;
        else
            throw std::runtime_error("Unsupported VFI TYPE!");
    }

    std::vector<VFIConfigurationFile::RawData> _load_data_from_yaml_file(const std::string& config_file)
    {
        if (!vfi_config_file_yaml_)
            vfi_config_file_yaml_ = std::make_shared<VFIConfigurationFileYaml>(config_file);
        return vfi_config_file_yaml_->get_raw_data();
    }

    std::string _extract_tag(const VFIConfigurationFile::RawData& raw_data) {
        return std::visit([](auto&& arg) -> std::string {
            return arg.tag;
        }, raw_data);
    }

    Impl()
    {

    };
};

RobotConstraintEditor::RobotConstraintEditor() {
    impl_ = std::make_shared<RobotConstraintEditor::Impl>();
}



void  RobotConstraintEditor::load_data_from_yaml_file(const std::string& config_file)
{
    std::vector<VFIConfigurationFile::RawData> raw_data = impl_->_load_data_from_yaml_file(config_file);
    impl_->yaml_raw_data_map_.clear();
    for (const auto& data : raw_data) {
        std::string tag = impl_->_extract_tag(data);
        impl_->yaml_raw_data_map_.try_emplace(tag, data);
    }
}

void RobotConstraintEditor::edit_data(const std::string& tag, const VFIConfigurationFile::RawData& data)
{
    if (impl_->vfi_config_file_yaml_)
    {
        try{
            // Check if tag exists
            auto old_it = impl_->yaml_raw_data_map_.find(tag);
            if (old_it == impl_->yaml_raw_data_map_.end())
                throw std::runtime_error("Tag '" + tag + "' not found!");

            auto current_data = impl_->yaml_raw_data_map_.at(tag);
            if (impl_->_get_vfi_type(current_data) != impl_->_get_vfi_type(data))
                throw std::runtime_error("The new data has a different VFI type from the current one!");

            impl_->yaml_raw_data_map_.erase(tag);
            impl_->yaml_raw_data_map_.try_emplace(impl_->_extract_tag(data), data);
        } catch (const std::runtime_error& e) {
            std::cerr<<e.what()<<std::endl;
            throw std::runtime_error("RobotConstraintEditor::edit_data: Fail to update the VFI data!");
        }

    }
}

std::vector<VFIConfigurationFile::RawData> RobotConstraintEditor::get_raw_data()
{
    std::vector<VFIConfigurationFile::RawData> raw_data;
    raw_data.reserve(impl_->yaml_raw_data_map_.size());
    for (auto& pair : impl_->yaml_raw_data_map_)
        raw_data.push_back(pair.second);
    return raw_data;
}

}
