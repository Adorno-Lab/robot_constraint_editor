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
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <variant>




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
    int vfi_file_version_ = 2; // Version of the data in the editor (2 or 3)
    bool zero_indexed_ = true; // default value
    std::shared_ptr<VFIConfigurationFile> interface_;

    std::map<std::string, VFIConfigurationFile::Data> yaml_raw_data_map_; // Version 2 data
    VFIConfigurationFile::DOCUMENT_V3 document_v3_;                       // Version 3 data

    /**
     * @brief _is_the_same_type checks if two RawData structures have the same type.
     * @param data1
     * @param data2
     * @return True if data1 and data2 have the same type. False otherwise.
     */
    bool _is_the_same_type(const VFIConfigurationFile::Data& data1, const VFIConfigurationFile::Data& data2)
    {
        return data1.index() == data2.index();
    }

    /**
     * @brief _extract_tag This method gets the tag of a RawData element.
     * @param raw_data
     * @return The desired tag
     */
    std::string _extract_tag(const VFIConfigurationFile::Data& raw_data) {
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

    /**
     * @brief _check_version throws an exception if the editor does not contain data of the given version.
     * @param version The required version.
     * @param method The name of the method that requires the version, used in the error message.
     */
    void _check_version(const int& version, const std::string& method) const
    {
        if (vfi_file_version_ != version)
            throw std::runtime_error("RobotConstraintEditor::" + method + " requires version "
                                     + std::to_string(version) + " data, but the editor contains version "
                                     + std::to_string(vfi_file_version_) + " data.");
    }

    std::string _extract_tag_v3(const VFIConfigurationFile::DataV3& data) const
    {
        return std::visit([](const auto& arg) -> std::string { return arg.tag; }, data);
    }

    /**
     * @brief _find_vfi returns the version 3 VFI with the given tag, or nullptr if it does not exist.
     */
    VFIConfigurationFile::DataV3* _find_vfi(const std::string& tag)
    {
        for (auto& data : document_v3_.vfi_array)
            if (_extract_tag_v3(data) == tag)
                return &data;
        return nullptr;
    }

    VFIConfigurationFile::DataV3& _get_vfi(const std::string& tag)
    {
        VFIConfigurationFile::DataV3* data = _find_vfi(tag);
        if (!data)
            throw std::runtime_error("Tag '" + tag + "' not found!");
        return *data;
    }

    /**
     * @brief _find_entity returns the entity with the given name, or nullptr if it does not exist.
     */
    template<typename ENTITY>
    ENTITY* _find_entity(std::vector<ENTITY>& entities, const std::string& name)
    {
        for (auto& entity : entities)
            if (entity.name == name)
                return &entity;
        return nullptr;
    }

    /**
     * @brief _check_name_available throws an exception if an entity already uses the name.
     */
    void _check_name_available(const std::string& name)
    {
        if (_find_entity(document_v3_.environment_entities, name) || _find_entity(document_v3_.robot_entities, name))
            throw std::runtime_error("Entity name '" + name + "' is being used!");
    }

    /**
     * @brief _find_user returns the tag of the first VFI that uses the entity, or an empty string if no VFI uses it.
     */
    std::string _find_user(const std::string& name)
    {
        for (const auto& data : document_v3_.vfi_array)
        {
            const bool used = std::visit([&name](const auto& arg) {
                auto contains = [&name](const std::vector<std::string>& list) {
                    return std::find(list.begin(), list.end(), name) != list.end();
                };
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3>)
                    return contains(arg.entity_environment) || contains(arg.entity_robot);
                else
                    return contains(arg.entity_one) || contains(arg.entity_two);
            }, data);
            if (used)
                return _extract_tag_v3(data);
        }
        return "";
    }

    /**
     * @brief _rename_references replaces the name of an entity in all the VFIs that use it.
     */
    void _rename_references(const std::string& name, const std::string& new_name)
    {
        for (auto& data : document_v3_.vfi_array)
        {
            std::visit([&](auto& arg) {
                auto rename = [&](std::vector<std::string>& list) {
                    std::replace(list.begin(), list.end(), name, new_name);
                };
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3>) {
                    rename(arg.entity_environment);
                    rename(arg.entity_robot);
                } else {
                    rename(arg.entity_one);
                    rename(arg.entity_two);
                }
            }, data);
        }
    }

    /**
     * @brief _edit_data_v3 modifies the value of a key in the version 3 VFI with the given tag.
     *        vfi_type cannot be edited. Use replace_data() to change the VFI type.
     */
    template<typename T>
    void _edit_data_v3(const std::string& tag, const std::string& key, const T& value)
    {
        VFIConfigurationFile::DataV3& data = _get_vfi(tag);

        if (key == "tag")
        {
            if constexpr (std::is_convertible_v<T, std::string>) {
                const std::string new_tag = value;
                if (new_tag != tag && _find_vfi(new_tag))
                    throw std::runtime_error("Tag '" + new_tag + "' is being used!");
                std::visit([&new_tag](auto& arg) { arg.tag = new_tag; }, data);
                return;
            } else {
                throw std::runtime_error("Tag must be convertible to string");
            }
        }
        if (key == "vfi_type")
            throw std::runtime_error("vfi_type cannot be edited. Use replace_data() to change the VFI type.");

        std::visit([&](auto& arg) {
            using DataType = std::decay_t<decltype(arg)>;

            // Helper function to assign value with type checking
            auto assign_if_match = [&](auto& field, const std::string& field_name) -> bool {
                if (key != field_name) return false;
                using FieldType = std::decay_t<decltype(field)>;
                if constexpr (std::is_convertible_v<T, FieldType>) {
                    field = value;  // Allow implicit conversions (int to double, etc.)
                    return true;
                } else {
                    throw std::runtime_error("Type mismatch for field '" + key +
                                             "'. Expected: " + typeid(FieldType).name() +
                                             ", Got: " + typeid(T).name());
                }
            };

            bool modified = assign_if_match(arg.direction, "direction")
                         || assign_if_match(arg.safe_distance, "safe_distance")
                         || assign_if_match(arg.buffer, "buffer")
                         || assign_if_match(arg.vfi_gain, "vfi_gain");

            if constexpr (std::is_same_v<DataType, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3>) {
                modified = modified
                        || assign_if_match(arg.entity_environment, "entity_environment")
                        || assign_if_match(arg.entity_robot, "entity_robot")
                        || assign_if_match(arg.entity_environment_primitive_type, "entity_environment_primitive_type")
                        || assign_if_match(arg.entity_robot_primitive_type, "entity_robot_primitive_type");
            } else {
                modified = modified
                        || assign_if_match(arg.entity_one, "entity_one")
                        || assign_if_match(arg.entity_two, "entity_two")
                        || assign_if_match(arg.entity_one_primitive_type, "entity_one_primitive_type")
                        || assign_if_match(arg.entity_two_primitive_type, "entity_two_primitive_type");
            }

            if (!modified)
                throw std::runtime_error("Key '" + key + "' not found for " + arg.vfi_type);
        }, data);
    }

    Impl()
    {

    };
};

/**
 * @brief RobotConstraintEditor::RobotConstraintEditor ctor of the class
 */
RobotConstraintEditor::RobotConstraintEditor(const std::shared_ptr<VFIConfigurationFile> &interface) {
    impl_ = std::make_shared<RobotConstraintEditor::Impl>();
    impl_->interface_ = interface;
}

/**
 * @brief RobotConstraintEditor::load_data loads a configuration file. A version 3 file replaces the
 *        content of the editor. The data of a version 2 file is added to the version 2 data of the editor.
 * @param config_file
 */
void RobotConstraintEditor::load_data(const std::string& config_file)
{
    if (impl_->interface_)
    {
        impl_->interface_->load_data(config_file);
        const VFIConfigurationFile::Document document = impl_->interface_->get_document();
        if (std::holds_alternative<VFIConfigurationFile::DOCUMENT_V3>(document))
        {
            set_document(std::get<VFIConfigurationFile::DOCUMENT_V3>(document));
            return;
        }
        if (impl_->vfi_file_version_ != 2)
        {
            impl_->vfi_file_version_ = 2;
            impl_->document_v3_ = VFIConfigurationFile::DOCUMENT_V3{};
        }
        add_data(impl_->interface_->get_data());
    }else
        throw std::runtime_error("The VFIConfigurationFile pointer is undefined!");
}

/**
 * @brief RobotConstraintEditor::add_data adds data to compose the YAML file.
 * @param vector_data A vector containing VFIConfigurationFile::RawData elements
 */
void  RobotConstraintEditor::add_data(const std::vector<VFIConfigurationFile::Data>& vector_data)
{
    for (auto& data : vector_data)
        add_data(data);
}

/**
 * @brief RobotConstraintEditor::replace_data removes the data stored in the corresponding tag, and adds
 *              the new data. The new data will will be tagged automatically.
 * @param tag The tag of the data to be removed
 * @param data The new data to add.
 */
void RobotConstraintEditor::replace_data(const std::string& tag, const VFIConfigurationFile::Data& data)
{
    impl_->_check_version(2, "replace_data");
    try{
        remove_data(tag);
        add_data(data);
    } catch (const std::runtime_error& e) {
        std::cerr<<e.what()<<std::endl;
        throw std::runtime_error("RobotConstraintEditor::edit_data: Fail to update the VFI data!");
    }
}

/**
 * @brief RobotConstraintEditor::add_data adds data to compose the YAML file.
 * @param data
 */
void RobotConstraintEditor::add_data(const VFIConfigurationFile::Data& data)
{
    impl_->_check_version(2, "add_data");
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
    if (impl_->vfi_file_version_ == 3)
    {
        auto& vfi_array = impl_->document_v3_.vfi_array;
        const auto it = std::find_if(vfi_array.begin(), vfi_array.end(),
                                     [&](const VFIConfigurationFile::DataV3& data) {
                                         return impl_->_extract_tag_v3(data) == tag; });
        if (it == vfi_array.end())
            throw std::runtime_error("Tag '" + tag + "' not found!");
        vfi_array.erase(it);
        return;
    }
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
    if (impl_->vfi_file_version_ == 3)
    {
        impl_->_edit_data_v3(tag, key, value);
        return;
    }

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

        if constexpr (std::is_same_v<DataType, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA>) {
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
            else if (assign_if_match(arg.buffer, "buffer")) modified = true;
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

        } else if constexpr (std::is_same_v<DataType, VFIConfigurationFile::ROBOT_TO_ROBOT_DATA>) {
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
            else if (assign_if_match(arg.buffer, "buffer")) modified = true;
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
    impl_->_check_version(2, "save_data");
    if (impl_->interface_)
    {
        std::vector<VFIConfigurationFile::Data> data;
        data.reserve(impl_->yaml_raw_data_map_.size());
        for (auto& pair : impl_->yaml_raw_data_map_)
            data.push_back(pair.second);

        impl_->interface_->save_data(data, vfi_file_version, zero_indexed, path_config_file);
    }else
        throw std::runtime_error("The VFIConfigurationFile pointer is undefined!");
}

/**
 * @brief RobotConstraintEditor::get_raw_data returns the raw data vector
 * @return The desired vector
 */
std::vector<VFIConfigurationFile::Data> RobotConstraintEditor::get_data()
{
    impl_->_check_version(2, "get_data");
    std::vector<VFIConfigurationFile::Data> raw_data;
    raw_data.reserve(impl_->yaml_raw_data_map_.size());
    for (auto& pair : impl_->yaml_raw_data_map_)
        raw_data.push_back(pair.second);
    return raw_data;
}

/**
 * @brief RobotConstraintEditor::get_vfi_file_version returns the version of the data in the editor.
 * @return 2 or 3.
 */
int RobotConstraintEditor::get_vfi_file_version() const
{
    return impl_->vfi_file_version_;
}

/**
 * @brief RobotConstraintEditor::set_document replaces the content of the editor with a version 3 document.
 *        The document is validated when it is saved (see validate()).
 * @param document The version 3 document.
 */
void RobotConstraintEditor::set_document(const VFIConfigurationFile::DOCUMENT_V3& document)
{
    impl_->vfi_file_version_ = 3;
    impl_->document_v3_ = document;
    impl_->yaml_raw_data_map_.clear();
}

/**
 * @brief RobotConstraintEditor::get_document returns the version 3 document of the editor.
 * @return The desired document.
 */
VFIConfigurationFile::DOCUMENT_V3 RobotConstraintEditor::get_document() const
{
    impl_->_check_version(3, "get_document");
    return impl_->document_v3_;
}

/**
 * @brief RobotConstraintEditor::validate checks the validation rules of the version 3 specification.
 *        It throws an exception describing the first rule that is not met.
 */
void RobotConstraintEditor::validate() const
{
    impl_->_check_version(3, "validate");
    VFIConfigurationFileV3::validate(impl_->document_v3_);
}

/**
 * @brief RobotConstraintEditor::save_document saves the version 3 document. It is validated before the
 *        file is written.
 * @param path_config_file The path to the YAML file, including its name and format.
 */
void RobotConstraintEditor::save_document(const std::string& path_config_file)
{
    impl_->_check_version(3, "save_document");
    if (!impl_->interface_)
        throw std::runtime_error("The VFIConfigurationFile pointer is undefined!");
    impl_->interface_->save_document(impl_->document_v3_, path_config_file);
}

/**
 * @brief RobotConstraintEditor::set_metadata sets the metadata of the version 3 document.
 */
void RobotConstraintEditor::set_metadata(const VFIConfigurationFile::METADATA& metadata)
{
    impl_->_check_version(3, "set_metadata");
    impl_->document_v3_.metadata = metadata;
}

/**
 * @brief RobotConstraintEditor::set_robot sets the robot of the version 3 document.
 */
void RobotConstraintEditor::set_robot(const VFIConfigurationFile::ROBOT& robot)
{
    impl_->_check_version(3, "set_robot");
    impl_->document_v3_.robots = {robot};
}

/**
 * @brief RobotConstraintEditor::add_data adds a version 3 VFI at the end of the vfi_array.
 * @param data The VFI. Its tag must not be used by another VFI.
 */
void RobotConstraintEditor::add_data(const VFIConfigurationFile::DataV3& data)
{
    impl_->_check_version(3, "add_data");
    const std::string tag = impl_->_extract_tag_v3(data);
    if (impl_->_find_vfi(tag))
        throw std::runtime_error("Tag '" + tag + "' is being used!");
    impl_->document_v3_.vfi_array.push_back(data);
}

/**
 * @brief RobotConstraintEditor::replace_data replaces the version 3 VFI with the given tag. The position
 *        of the VFI in the vfi_array is kept.
 * @param tag The tag of the VFI to be replaced.
 * @param data The new VFI. Its tag must not be used by another VFI.
 */
void RobotConstraintEditor::replace_data(const std::string& tag, const VFIConfigurationFile::DataV3& data)
{
    impl_->_check_version(3, "replace_data");
    VFIConfigurationFile::DataV3& current = impl_->_get_vfi(tag);
    const std::string new_tag = impl_->_extract_tag_v3(data);
    if (new_tag != tag && impl_->_find_vfi(new_tag))
        throw std::runtime_error("Tag '" + new_tag + "' is being used!");
    current = data;
}

/**
 * @brief RobotConstraintEditor::add_environment_entity adds an environment entity.
 * @param entity The entity. Its name must not be used by another entity.
 */
void RobotConstraintEditor::add_environment_entity(const VFIConfigurationFile::ENVIRONMENT_ENTITY& entity)
{
    impl_->_check_version(3, "add_environment_entity");
    impl_->_check_name_available(entity.name);
    impl_->document_v3_.environment_entities.push_back(entity);
}

/**
 * @brief RobotConstraintEditor::add_robot_entity adds a robot entity.
 * @param entity The entity. Its name must not be used by another entity.
 */
void RobotConstraintEditor::add_robot_entity(const VFIConfigurationFile::ROBOT_ENTITY& entity)
{
    impl_->_check_version(3, "add_robot_entity");
    impl_->_check_name_available(entity.name);
    impl_->document_v3_.robot_entities.push_back(entity);
}

/**
 * @brief RobotConstraintEditor::replace_environment_entity replaces an environment entity. If the name
 *        changes, the VFIs that use the entity are updated.
 * @param name The name of the entity to be replaced.
 * @param entity The new entity.
 */
void RobotConstraintEditor::replace_environment_entity(const std::string& name,
                                                       const VFIConfigurationFile::ENVIRONMENT_ENTITY& entity)
{
    impl_->_check_version(3, "replace_environment_entity");
    auto* current = impl_->_find_entity(impl_->document_v3_.environment_entities, name);
    if (!current)
        throw std::runtime_error("Environment entity '" + name + "' not found!");
    if (entity.name != name)
    {
        impl_->_check_name_available(entity.name);
        impl_->_rename_references(name, entity.name);
    }
    *current = entity;
}

/**
 * @brief RobotConstraintEditor::replace_robot_entity replaces a robot entity. If the name changes,
 *        the VFIs that use the entity are updated.
 * @param name The name of the entity to be replaced.
 * @param entity The new entity.
 */
void RobotConstraintEditor::replace_robot_entity(const std::string& name,
                                                 const VFIConfigurationFile::ROBOT_ENTITY& entity)
{
    impl_->_check_version(3, "replace_robot_entity");
    auto* current = impl_->_find_entity(impl_->document_v3_.robot_entities, name);
    if (!current)
        throw std::runtime_error("Robot entity '" + name + "' not found!");
    if (entity.name != name)
    {
        impl_->_check_name_available(entity.name);
        impl_->_rename_references(name, entity.name);
    }
    *current = entity;
}

/**
 * @brief RobotConstraintEditor::rename_entity renames an entity and updates the VFIs that use it.
 * @param name The name of the entity.
 * @param new_name The new name. It must not be used by another entity.
 */
void RobotConstraintEditor::rename_entity(const std::string& name, const std::string& new_name)
{
    impl_->_check_version(3, "rename_entity");
    auto* environment_entity = impl_->_find_entity(impl_->document_v3_.environment_entities, name);
    auto* robot_entity = impl_->_find_entity(impl_->document_v3_.robot_entities, name);
    if (!environment_entity && !robot_entity)
        throw std::runtime_error("Entity '" + name + "' not found!");
    if (new_name == name)
        return;
    impl_->_check_name_available(new_name);
    impl_->_rename_references(name, new_name);
    if (environment_entity)
        environment_entity->name = new_name;
    else
        robot_entity->name = new_name;
}

/**
 * @brief RobotConstraintEditor::remove_entity removes an entity. Entities used by a VFI cannot be removed.
 * @param name The name of the entity.
 */
void RobotConstraintEditor::remove_entity(const std::string& name)
{
    impl_->_check_version(3, "remove_entity");
    const std::string user = impl_->_find_user(name);
    if (!user.empty())
        throw std::runtime_error("Entity '" + name + "' is used by the VFI with tag '" + user + "'!");

    auto erase_entity = [&name](auto& entities) {
        const auto it = std::find_if(entities.begin(), entities.end(),
                                     [&name](const auto& entity) { return entity.name == name; });
        if (it == entities.end())
            return false;
        entities.erase(it);
        return true;
    };
    if (!erase_entity(impl_->document_v3_.environment_entities) && !erase_entity(impl_->document_v3_.robot_entities))
        throw std::runtime_error("Entity '" + name + "' not found!");
}


}
