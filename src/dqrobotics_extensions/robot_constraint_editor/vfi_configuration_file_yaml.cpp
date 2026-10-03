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
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <array>
#include <charconv>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <system_error>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <dqrobotics_extensions/robot_constraint_editor/utils.hpp>

namespace DQ_robotics_extensions
{

class VFIConfigurationFileYaml::Impl
{
public:
    static constexpr int default_vfi_file_version_ = 2;
    static constexpr bool default_zero_indexed_ = true;

    YAML::Node config_;
    std::string config_file_;
    int vfi_file_version_ = default_vfi_file_version_;
    bool zero_indexed_ = default_zero_indexed_;
    std::vector<Data> raw_data_;
    Document document_;
    bool document_loaded_ = false;

    // Pose forms found while parsing a version 3 file
    bool mapping_pose_found_ = false;
    bool sequence_pose_found_ = false;

    Impl()
    {

    };

    /**
     * @brief _get_value returns the value of a required key.
     * @param node A YAML mapping.
     * @param key The key.
     * @param context The location of the node, used in the error messages.
     * @return The desired value.
     */
    template<typename T>
    T _get_value(const YAML::Node& node, const std::string& key, const std::string& context)
    {
        const YAML::Node value = node[key];
        if (!value)
            throw std::runtime_error(context + ": missing required key '" + key + "'.");
        try {
            return value.as<T>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(context + ": invalid value for '" + key + "'.");
        }
    }

    /**
     * @brief _get_value returns the value of an optional key, or default_value if the key is not defined.
     */
    template<typename T>
    T _get_value(const YAML::Node& node, const std::string& key, const std::string& context,
                 const T& default_value)
    {
        if (!node[key])
            return default_value;
        return _get_value<T>(node, key, context);
    }

    /**
     * @brief _get_sequence returns the YAML sequence of a key.
     * @param node A YAML mapping.
     * @param key The key.
     * @param context The location of the node, used in the error messages.
     * @param required If false, an empty sequence is returned when the key is not defined.
     * @return The desired sequence.
     */
    YAML::Node _get_sequence(const YAML::Node& node, const std::string& key, const std::string& context,
                             const bool& required = true)
    {
        const YAML::Node value = node[key];
        if (!value)
        {
            if (required)
                throw std::runtime_error(context + ": missing required key '" + key + "'.");
            return YAML::Node(YAML::NodeType::Sequence);
        }
        if (!value.IsSequence())
            throw std::runtime_error(context + ": '" + key + "' must be a list.");
        return value;
    }

    /**
     * @brief _get_string_list returns the string list of a required key.
     */
    std::vector<std::string> _get_string_list(const YAML::Node& node, const std::string& key,
                                              const std::string& context)
    {
        const YAML::Node sequence = _get_sequence(node, key, context);
        try {
            return sequence.as<std::vector<std::string>>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(context + ": '" + key + "' must be a list of strings.");
        }
    }

    /**
     * @brief _check_map throws an exception if the node is not a YAML mapping.
     */
    void _check_map(const YAML::Node& node, const std::string& context)
    {
        if (!node.IsMap())
            throw std::runtime_error(context + ": must be a mapping.");
    }

    /**
     * @brief _to_array converts a YAML sequence of N numbers into an array.
     * @param node The YAML sequence.
     * @param context The location of the node, used in the error messages.
     * @return The desired array.
     */
    template<std::size_t N>
    std::array<double, N> _to_array(const YAML::Node& node, const std::string& context)
    {
        if (!node)
            throw std::runtime_error(context + ": missing.");
        if (!node.IsSequence() || node.size() != N)
            throw std::runtime_error(context + ": must be a list of " + std::to_string(N) + " numbers.");
        std::array<double, N> values;
        try {
            for (std::size_t i = 0; i < N; ++i)
                values.at(i) = node[i].as<double>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(context + ": must be a list of " + std::to_string(N) + " numbers.");
        }
        return values;
    }



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
     * @brief VFIConfigurationFileYaml::_extract_yaml_data reads the YAML file and stores its content.
     *        Version 3 files are parsed by _parse_v3(). Other versions are parsed by _parse_v2().
     */
    void _extract_yaml_data()
    {
        raw_data_.clear();
        document_ = DOCUMENT_V2{};
        document_loaded_ = false;
        vfi_file_version_ = default_vfi_file_version_;
        zero_indexed_ = default_zero_indexed_;
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



            if (vfi_file_version_ == 3)
                _parse_v3(config_);
            else
                _parse_v2(config_);
            document_loaded_ = true;
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
     * @brief _parse_v2 parses the vfi_array of a version 2 file.
     * @param config The root node of the YAML file.
     */
    void _parse_v2(const YAML::Node& config)
    {
        const YAML::Node& vfi_array = config["vfi_array"]; //Aliasing

        for (const auto& parameter : vfi_array) {
            try {
                std::string vfi_type = parameter["vfi_type"].as<std::string>();

                if (vfi_type == "ENVIRONMENT_TO_ROBOT") {
                    ENVIRONMENT_TO_ROBOT_DATA env_data;
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
                    try {
                        env_data.buffer = parameter["buffer"].as<double>();
                    } catch (...) {
                        // Use the default buffer value defined in the virtual class
                        DQ_robotics_extensions::VFIConfigurationFile::BASE_DATA data;
                        env_data.buffer = data.buffer;
                    }

                    env_data.vfi_gain = parameter["vfi_gain"].as<double>();
                    env_data.direction = parameter["direction"].as<std::string>();
                    env_data.tag = parameter["tag"].as<std::string>();
                    raw_data_.push_back(env_data);

                }else if (vfi_type == "ROBOT_TO_ROBOT") {
                    ROBOT_TO_ROBOT_DATA robot_data;
                    robot_data.vfi_type = vfi_type;
                    robot_data.cs_entity_one  = get_vector_list(parameter["cs_entity_one"],
                                                                      "cs_entity_one");
                    robot_data.cs_entity_two = get_vector_list(parameter["cs_entity_two"],
                                                                      "cs_entity_two");
                    robot_data.entity_one_primitive_type = parameter["entity_one_primitive_type"].as<std::string>();
                    robot_data.entity_two_primitive_type = parameter["entity_two_primitive_type"].as<std::string>();
                    robot_data.robot_index_one = parameter["robot_index_one"].as<int>();
                    robot_data.robot_index_two = parameter["robot_index_two"].as<int>();
                    robot_data.joint_index_one = parameter["joint_index_one"].as<int>();
                    robot_data.joint_index_two = parameter["joint_index_two"].as<int>();
                    robot_data.safe_distance = parameter["safe_distance"].as<double>();
                    try {
                        robot_data.buffer = parameter["buffer"].as<double>();
                    } catch (...) {
                        // Use the default buffer value defined in the virtual class
                        DQ_robotics_extensions::VFIConfigurationFile::BASE_DATA data;
                        robot_data.buffer = data.buffer;
                    }
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
                throw std::runtime_error(e.msg);
            }
        }
        document_ = DOCUMENT_V2{zero_indexed_, raw_data_};
    }

    /**
     * @brief _parse_pose parses a pose or offset written in either of the two forms of Section 5.1.
     * @param node The YAML mapping that contains the key.
     * @param key "pose" or "offset".
     * @param context The location of the node, used in the error messages.
     * @return The desired POSE. The values are not normalized.
     */
    POSE _parse_pose(const YAML::Node& node, const std::string& key, const std::string& context)
    {
        const YAML::Node value = node[key];
        const std::string pose_context = context + ", " + key;
        if (!value)
            throw std::runtime_error(context + ": missing required key '" + key + "'.");

        if (value.IsMap())
        {
            mapping_pose_found_ = true;
            POSE pose;
            pose.translation = _to_array<3>(value["translation"], pose_context + ", translation");
            pose.rotation = _to_array<4>(value["rotation"], pose_context + ", rotation");
            return pose;
        }
        if (value.IsSequence())
        {
            sequence_pose_found_ = true;
            const std::array<double, 8> vec8 = _to_array<8>(value, pose_context);
            try {
                return VFIConfigurationFileV3::dq_to_pose(
                    DQ_robotics::DQ(Eigen::Map<const Eigen::VectorXd>(vec8.data(), 8)));
            } catch (const std::runtime_error& e) {
                throw std::runtime_error(pose_context + ": " + e.what());
            }
        }
        throw std::runtime_error(pose_context + ": must be a mapping with translation and rotation, "
                                                "or a list of 8 numbers.");
    }

    /**
     * @brief _parse_base_data parses the parameters shared by all VFI types.
     */
    void _parse_base_data(const YAML::Node& node, BASE_DATA& data, const std::string& context)
    {
        data.vfi_type = _get_value<std::string>(node, "vfi_type", context);
        data.safe_distance = _get_value<double>(node, "safe_distance", context);
        data.buffer = _get_value<double>(node, "buffer", context, data.buffer);
        data.vfi_gain = _get_value<double>(node, "vfi_gain", context);
        data.direction = _get_value<std::string>(node, "direction", context);
        data.tag = _get_value<std::string>(node, "tag", context);
    }

    /**
     * @brief _parse_v3 parses a version 3 file and validates it (Section 9).
     * @param config The root node of the YAML file.
     */
    void _parse_v3(const YAML::Node& config)
    {
        mapping_pose_found_ = false;
        sequence_pose_found_ = false;

        DOCUMENT_V3 document;
        document.zero_indexed = _get_value<bool>(config, "zero_indexed", "header");

        // metadata (optional)
        if (config["metadata"])
        {
            const YAML::Node metadata = config["metadata"];
            _check_map(metadata, "metadata");
            document.metadata.description = _get_value<std::string>(metadata, "description", "metadata", "");
            document.metadata.generated_by = _get_value<std::string>(metadata, "generated_by", "metadata", "");
            document.metadata.source = _get_value<std::string>(metadata, "source", "metadata", "");
        }

        // robots
        const YAML::Node robots = _get_sequence(config, "robots", "header");
        for (std::size_t i = 0; i < robots.size(); ++i)
        {
            const std::string context = "robots[" + std::to_string(i) + "]";
            const YAML::Node item = robots[i];
            _check_map(item, context);
            ROBOT robot;
            robot.robot_index = _get_value<int>(item, "robot_index", context);
            robot.name = _get_value<std::string>(item, "name", context);
            robot.dim_configuration = _get_value<int>(item, "dim_configuration", context);
            document.robots.push_back(robot);
        }

        // environment_entities (optional)
        const YAML::Node environment_entities = _get_sequence(config, "environment_entities", "header", false);
        for (std::size_t i = 0; i < environment_entities.size(); ++i)
        {
            const std::string context = "environment_entities[" + std::to_string(i) + "]";
            const YAML::Node item = environment_entities[i];
            _check_map(item, context);
            ENVIRONMENT_ENTITY entity;
            entity.name = _get_value<std::string>(item, "name", context);
            const std::string entity_context = context + " ('" + entity.name + "')";
            entity.pose = _parse_pose(item, "pose", entity_context);
            entity.attached_direction = _get_value<std::string>(item, "attached_direction", entity_context,
                                                                entity.attached_direction);
            document.environment_entities.push_back(entity);
        }

        // robot_entities
        const YAML::Node robot_entities = _get_sequence(config, "robot_entities", "header");
        for (std::size_t i = 0; i < robot_entities.size(); ++i)
        {
            const std::string context = "robot_entities[" + std::to_string(i) + "]";
            const YAML::Node item = robot_entities[i];
            _check_map(item, context);
            ROBOT_ENTITY entity;
            entity.name = _get_value<std::string>(item, "name", context);
            const std::string entity_context = context + " ('" + entity.name + "')";
            entity.robot_index = _get_value<int>(item, "robot_index", entity_context);
            entity.joint_index = _get_value<int>(item, "joint_index", entity_context);
            entity.offset = _parse_pose(item, "offset", entity_context);
            entity.attached_direction = _get_value<std::string>(item, "attached_direction", entity_context,
                                                                entity.attached_direction);
            document.robot_entities.push_back(entity);
        }

        // vfi_array
        const YAML::Node vfi_array = _get_sequence(config, "vfi_array", "header");
        for (std::size_t i = 0; i < vfi_array.size(); ++i)
        {
            std::string context = "vfi_array[" + std::to_string(i) + "]";
            const YAML::Node item = vfi_array[i];
            _check_map(item, context);
            context += " (tag '" + _get_value<std::string>(item, "tag", context) + "')";
            const std::string vfi_type = _get_value<std::string>(item, "vfi_type", context);

            if (vfi_type == "ENVIRONMENT_TO_ROBOT") {
                ENVIRONMENT_TO_ROBOT_DATA_V3 env_data;
                _parse_base_data(item, env_data, context);
                env_data.entity_environment = _get_string_list(item, "entity_environment", context);
                env_data.entity_robot = _get_string_list(item, "entity_robot", context);
                env_data.entity_environment_primitive_type =
                    _get_value<std::string>(item, "entity_environment_primitive_type", context);
                env_data.entity_robot_primitive_type =
                    _get_value<std::string>(item, "entity_robot_primitive_type", context);
                document.vfi_array.push_back(env_data);

            }else if (vfi_type == "ROBOT_TO_ROBOT") {
                ROBOT_TO_ROBOT_DATA_V3 robot_data;
                _parse_base_data(item, robot_data, context);
                robot_data.entity_one = _get_string_list(item, "entity_one", context);
                robot_data.entity_two = _get_string_list(item, "entity_two", context);
                robot_data.entity_one_primitive_type =
                    _get_value<std::string>(item, "entity_one_primitive_type", context);
                robot_data.entity_two_primitive_type =
                    _get_value<std::string>(item, "entity_two_primitive_type", context);
                document.vfi_array.push_back(robot_data);

            }else {
                throw std::runtime_error(context + ": unknown vfi_type '" + vfi_type + "'.");
            }
        }

        // The mapping form is the default (Section 5.1.4). The sequence form is kept
        // only if the whole file uses it.
        document.pose_format = (sequence_pose_found_ && !mapping_pose_found_)
                                   ? POSE_FORMAT::UNIT_DUAL_QUATERNION
                                   : POSE_FORMAT::TRANSLATION_ROTATION;

        VFIConfigurationFileV3::validate(document);
        document_ = document;
    }

    /**
     * @brief _to_yaml_double returns the shortest representation of a double that is recovered
     *        exactly when it is read (Section 2.1). Integral values are written with a decimal point.
     */
    std::string _to_yaml_double(const double& value)
    {
        std::array<char, 32> buffer;
        const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
        if (result.ec != std::errc())
            throw std::runtime_error("Cannot convert the value into a string.");
        std::string text(buffer.data(), result.ptr);
        if (text.find_first_of(".eEn") == std::string::npos) // 'n' detects nan and inf
            text += ".0";
        return text;
    }

    /**
     * @brief _quote returns a YAML double-quoted string.
     */
    std::string _quote(const std::string& text)
    {
        std::string quoted = "\"";
        for (const char& c : text)
        {
            switch (c) {
            case '"':  quoted += "\\\""; break;
            case '\\': quoted += "\\\\"; break;
            case '\n': quoted += "\\n"; break;
            case '\t': quoted += "\\t"; break;
            default:   quoted += c;
            }
        }
        return quoted + "\"";
    }

    /**
     * @brief _flow_list returns a YAML flow sequence of quoted strings (e.g., ["a", "b"]).
     */
    std::string _flow_list(const std::vector<std::string>& values)
    {
        std::string list = "[";
        for (std::size_t i = 0; i < values.size(); ++i)
            list += (i > 0 ? ", " : "") + _quote(values.at(i));
        return list + "]";
    }

    /**
     * @brief _flow_list returns a YAML flow sequence of numbers (e.g., [1.0, 0.5]).
     */
    template<std::size_t N>
    std::string _flow_list(const std::array<double, N>& values)
    {
        std::string list = "[";
        for (std::size_t i = 0; i < N; ++i)
            list += (i > 0 ? ", " : "") + _to_yaml_double(values.at(i));
        return list + "]";
    }

    /**
     * @brief _write_pose writes a pose or offset in the form given by format (Section 5.1).
     * @param out The output stream.
     * @param key "pose" or "offset".
     * @param pose The pose.
     * @param format The form used to write the pose.
     */
    void _write_pose(std::ostream& out, const std::string& key, const POSE& pose, const POSE_FORMAT& format)
    {
        if (format == POSE_FORMAT::UNIT_DUAL_QUATERNION)
        {
            const Eigen::Matrix<double, 8, 1> x = DQ_robotics::vec8(VFIConfigurationFileV3::pose_to_dq(pose));
            std::array<double, 8> coefficients;
            for (std::size_t i = 0; i < coefficients.size(); ++i)
                coefficients.at(i) = x(i);
            out << "    " << key << ": " << _flow_list(coefficients) << "\n";
        }
        else
        {
            out << "    " << key << ":\n";
            out << "      translation: " << _flow_list(pose.translation) << "\n";
            out << "      rotation:    " << _flow_list(pose.rotation) << "\n";
        }
    }

    /**
     * @brief _write_vfi_parameters writes the parameters shared by all VFI types, except vfi_type.
     */
    void _write_vfi_parameters(std::ostream& out, const BASE_DATA& data)
    {
        out << "    safe_distance: " << _to_yaml_double(data.safe_distance) << "\n";
        out << "    buffer: " << _to_yaml_double(data.buffer) << "\n";
        out << "    vfi_gain: " << _to_yaml_double(data.vfi_gain) << "\n";
        out << "    direction: " << _quote(data.direction) << "\n";
        out << "    tag: " << _quote(data.tag) << "\n";
    }

    /**
     * @brief _write_v3 writes a version 3 file using the writing style of Section 2.1.
     * @param document The document to write. It must be valid.
     * @param out The output stream.
     */
    void _write_v3(const DOCUMENT_V3& document, std::ostream& out)
    {
        out << "vfi_file_version: 3\n";
        out << "zero_indexed: " << bool2string(document.zero_indexed) << "\n";

        // metadata (optional): only the fields that are defined
        const METADATA& metadata = document.metadata;
        if (!metadata.description.empty() || !metadata.generated_by.empty() || !metadata.source.empty())
        {
            out << "\nmetadata:\n";
            if (!metadata.description.empty())
                out << "  description: " << _quote(metadata.description) << "\n";
            if (!metadata.generated_by.empty())
                out << "  generated_by: " << _quote(metadata.generated_by) << "\n";
            if (!metadata.source.empty())
                out << "  source: " << _quote(metadata.source) << "\n";
        }

        out << "\nrobots:\n";
        for (const auto& robot : document.robots)
        {
            out << "  -\n";
            out << "    robot_index: " << robot.robot_index << "\n";
            out << "    name: " << _quote(robot.name) << "\n";
            out << "    dim_configuration: " << robot.dim_configuration << "\n";
        }

        // environment_entities (optional)
        if (!document.environment_entities.empty())
        {
            out << "\nenvironment_entities:\n";
            for (const auto& entity : document.environment_entities)
            {
                out << "  -\n";
                out << "    name: " << _quote(entity.name) << "\n";
                _write_pose(out, "pose", entity.pose, document.pose_format);
                out << "    attached_direction: " << _quote(entity.attached_direction) << "\n";
            }
        }

        out << "\nrobot_entities:" << (document.robot_entities.empty() ? " []\n" : "\n");
        for (const auto& entity : document.robot_entities)
        {
            out << "  -\n";
            out << "    name: " << _quote(entity.name) << "\n";
            out << "    robot_index: " << entity.robot_index << "\n";
            out << "    joint_index: " << entity.joint_index << "\n";
            _write_pose(out, "offset", entity.offset, document.pose_format);
            out << "    attached_direction: " << _quote(entity.attached_direction) << "\n";
        }

        out << "\nvfi_array:" << (document.vfi_array.empty() ? " []\n" : "\n");
        for (const auto& vfi : document.vfi_array)
        {
            out << "  -\n";
            std::visit([&](const auto& data) {
                using T = std::decay_t<decltype(data)>;
                out << "    vfi_type: " << _quote(data.vfi_type) << "\n";
                if constexpr (std::is_same_v<T, ENVIRONMENT_TO_ROBOT_DATA_V3>) {
                    out << "    entity_environment: " << _flow_list(data.entity_environment) << "\n";
                    out << "    entity_robot: " << _flow_list(data.entity_robot) << "\n";
                    out << "    entity_environment_primitive_type: " << _quote(data.entity_environment_primitive_type) << "\n";
                    out << "    entity_robot_primitive_type: " << _quote(data.entity_robot_primitive_type) << "\n";
                } else if constexpr (std::is_same_v<T, ROBOT_TO_ROBOT_DATA_V3>) {
                    out << "    entity_one: " << _flow_list(data.entity_one) << "\n";
                    out << "    entity_two: " << _flow_list(data.entity_two) << "\n";
                    out << "    entity_one_primitive_type: " << _quote(data.entity_one_primitive_type) << "\n";
                    out << "    entity_two_primitive_type: " << _quote(data.entity_two_primitive_type) << "\n";
                }
                _write_vfi_parameters(out, data);
            }, vfi);
        }
    }

    /**
     * @brief _write_file writes the content into a file. The directory is created if it does not exist.
     */
    void _write_file(const std::string& content, const std::string& config_file)
    {
        if (config_file.empty())
            throw std::runtime_error("config_file path cannot be empty!");

        const std::filesystem::path directory = std::filesystem::path(config_file).parent_path();
        if (!directory.empty() && !std::filesystem::exists(directory)) {
            std::cout << "Creating directory: " << directory << std::endl;
            std::filesystem::create_directories(directory);
        }

        std::ofstream file(config_file);
        if (!file.is_open())
            throw std::runtime_error("Cannot open file for writing: " + config_file);
        file << content;
        file.close();
        if (file.fail())
            throw std::runtime_error("Cannot write the file: " + config_file);
    }

};

/**
 * @brief VFIConfigurationFileYaml::VFIConfigurationFileYaml ctor of the class.
 * @param config_file The configuration YAML file. This path must contain the file and its format.
 *                    Example: "/path_to_the_file/config_file.yaml"
 */
VFIConfigurationFileYaml::VFIConfigurationFileYaml()
{
    impl_ = std::make_shared<VFIConfigurationFileYaml::Impl>();
    //impl_->config_file_ = config_file;
    //impl_->_extract_yaml_data();
}

/**
 * @brief VFIConfigurationFileYaml::load_data loads a configuration file.
 * @param config_file The name of the file including its path and format.
 */
void VFIConfigurationFileYaml::load_data(const std::string& config_file)
{
    impl_->config_file_ = config_file;
    impl_->_extract_yaml_data();
}



/**
 * @brief VFIConfigurationFileYaml::get_raw_data gets the raw data vector from a YAML file.
 * @return A raw data vector.
 */
std::vector<VFIConfigurationFile::Data> VFIConfigurationFileYaml::get_data() const
{
    if (std::holds_alternative<DOCUMENT_V3>(impl_->document_))
        throw std::runtime_error("get_data() supports only version 2 files. Use get_document() instead.");
    if (impl_->raw_data_.empty())
        throw std::runtime_error("The vector data is empty!");
    return impl_->raw_data_;
}


/**
 * @brief VFIConfigurationFileYaml::get_document gets the complete content of the loaded configuration file.
 * @return A DOCUMENT_V2 or a DOCUMENT_V3, depending on the file version.
 */
VFIConfigurationFile::Document VFIConfigurationFileYaml::get_document() const
{
    if (!impl_->document_loaded_)
        throw std::runtime_error("No configuration file has been loaded!");
    return impl_->document_;
}


/**
 * @brief VFIConfigurationFileYaml::get_vfi_file_version gets the vfi_file_version data from
 *              the YAML file.
 * @return The desired data.
 */
int VFIConfigurationFileYaml::get_vfi_file_version() const
{
    return impl_->vfi_file_version_;
}


/**
 * @brief VFIConfigurationFileYaml::is_zero_indexed.
 * @return Returns true if the configuration file uses a zero-indexed convention to
 *         describe the joint and robot indexes. False otherwise.
 */
bool VFIConfigurationFileYaml::is_zero_indexed() const
{
    return impl_->zero_indexed_;
}

/**
 * @brief VFIConfigurationFileYaml::save_document saves a configuration file. The version is given by the
 *        document type. A DOCUMENT_V2 is saved with save_data(). A DOCUMENT_V3 is validated before the
 *        file is opened, so an invalid document does not modify the file.
 * @param document The DOCUMENT_V2 or DOCUMENT_V3 to save.
 * @param config_file The desired name of the file including its path and format.
 */
void VFIConfigurationFileYaml::save_document(const Document& document, const std::string& config_file)
{
    if (const auto* document_v2 = std::get_if<DOCUMENT_V2>(&document))
    {
        save_data(document_v2->vfi_array, 2, document_v2->zero_indexed, config_file);
        return;
    }

    try {
        const DOCUMENT_V3& document_v3 = std::get<DOCUMENT_V3>(document);
        VFIConfigurationFileV3::validate(document_v3);

        std::ostringstream content;
        impl_->_write_v3(document_v3, content);
        impl_->_write_file(content.str(), config_file);

        std::cout << "Successfully saved " << document_v3.vfi_array.size()
                  << " VFI entries to: " << config_file << std::endl;

    } catch (const std::filesystem::filesystem_error& e) {
        throw std::runtime_error("Filesystem error in save_document: " + std::string(e.what()));
    } catch (const std::exception& e) {
        throw std::runtime_error("Error in save_document: " + std::string(e.what()));
    }
}


/**
 * @brief VFIConfigurationFileYaml::save_data saves a configuration file containing the VFI constraints.
 * @param data the vector that contains the VFI configurations
 * @param The desired name of the file including its path and format.
 */
void VFIConfigurationFileYaml::save_data(const std::vector<Data> &data,
                                         const int &vfi_file_version,
                                         const bool &zero_indexed,
                                         const std::string &config_file)
{
    try {
        if (config_file.empty())
            throw std::runtime_error("config_file path cannot be empty!");

        // Create directory if it doesn't exist
        std::filesystem::path file_path(config_file);
        std::filesystem::path directory = file_path.parent_path();

        if (!directory.empty() && !std::filesystem::exists(directory)) {
            std::cout << "Creating directory: " << directory << std::endl;
            std::filesystem::create_directories(directory);
        }

        std::ofstream file(config_file);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + config_file);
        }

        // Write header using provided parameters
        file << "vfi_file_version: " << vfi_file_version << "\n";
        file << "zero_indexed: " << (zero_indexed ? "true" : "false") << "\n";
        file << "vfi_array:\n";

        // Write each data entry from the provided vector
        for (const auto& item : data) {
            file << "  -\n";
            std::visit([&file](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA>) {
                    file << "    vfi_type: \"" << arg.vfi_type << "\"\n";

                    // cs_entity_environment
                    file << "    cs_entity_environment: [";
                    for (size_t i = 0; i < arg.cs_entity_environment.size(); ++i) {
                        file << "\"" << arg.cs_entity_environment.at(i) << "\"";
                        if (i < arg.cs_entity_environment.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    // cs_entity_robot
                    file << "    cs_entity_robot: [";
                    for (size_t i = 0; i < arg.cs_entity_robot.size(); ++i) {
                        file << "\"" << arg.cs_entity_robot.at(i) << "\"";
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
                    file << "    buffer: " << arg.buffer << "\n";

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

                } else if constexpr (std::is_same_v<T, VFIConfigurationFile::ROBOT_TO_ROBOT_DATA>) {
                    file << "    vfi_type: \"" << arg.vfi_type << "\"\n";

                    // cs_entity_one
                    file << "    cs_entity_one: [";
                    for (size_t i = 0; i < arg.cs_entity_one.size(); ++i) {
                        file << "\"" << arg.cs_entity_one.at(i) << "\"";
                        if (i < arg.cs_entity_one.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    // cs_entity_two
                    file << "    cs_entity_two: [";
                    for (size_t i = 0; i < arg.cs_entity_two.size(); ++i) {
                        file << "\"" << arg.cs_entity_two.at(i) << "\"";
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
                    file << "    buffer: " << arg.buffer << "\n";

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
            }, item);
        }

        file.close();

        std::cout << "Successfully saved " << data.size()
                  << " VFI entries to: " << config_file << std::endl;

    } catch (const std::filesystem::filesystem_error& e) {
        throw std::runtime_error("Filesystem error in save_data: " + std::string(e.what()));
    } catch (const std::exception& e) {
        throw std::runtime_error("Error in save_data: " + std::string(e.what()));
    }
}


}
