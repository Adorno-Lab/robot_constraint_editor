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

#pragma once
#include <array>
#include <string>
#include <vector>
#include <variant>

namespace DQ_robotics_extensions
{

class VFIConfigurationFile
{
public:

    struct BASE_DATA{
        std::string vfi_type;
        double safe_distance;
        double buffer = 0.0;  //Default value
        double vfi_gain;
        std::string direction;
        std::string tag;

    };
    struct ENVIRONMENT_TO_ROBOT_DATA : BASE_DATA{
        std::vector<std::string> cs_entity_environment;
        std::vector<std::string> cs_entity_robot;
        std::string entity_environment_primitive_type;
        std::string entity_robot_primitive_type;
        int robot_index;
        int joint_index;

    };
    struct ROBOT_TO_ROBOT_DATA : BASE_DATA{
        std::vector<std::string> cs_entity_one;
        std::vector<std::string> cs_entity_two;
        std::string entity_one_primitive_type;
        std::string entity_two_primitive_type;
        int robot_index_one;
        int robot_index_two;
        int joint_index_one;
        int joint_index_two;
    };

    using Data = std::variant<ENVIRONMENT_TO_ROBOT_DATA, ROBOT_TO_ROBOT_DATA>;

    // Version 3 types. See design/specs_document/config_file_specification_v3.md

    // Form used to write the poses/offsets in the file (Section 5.1)
    enum class POSE_FORMAT{
        TRANSLATION_ROTATION,   // translation: [x, y, z], rotation: [w, x, y, z]
        UNIT_DUAL_QUATERNION    // [c1, c2, c3, c4, c5, c6, c7, c8]
    };
    struct POSE{
        std::array<double, 3> translation; // vec3(t)
        std::array<double, 4> rotation;    // vec4(r)
    };
    struct METADATA{
        std::string description;
        std::string generated_by;
        std::string source;
    };
    struct ROBOT{
        int robot_index;
        std::string name;
        int dim_configuration;
    };
    struct ENVIRONMENT_ENTITY{
        std::string name;
        POSE pose;
        std::string attached_direction = "k_"; //Default value
    };
    struct ROBOT_ENTITY{
        std::string name;
        int robot_index;
        int joint_index;
        POSE offset;
        std::string attached_direction = "k_"; //Default value
    };
    struct ENVIRONMENT_TO_ROBOT_DATA_V3 : BASE_DATA{
        std::vector<std::string> entity_environment;
        std::vector<std::string> entity_robot;
        std::string entity_environment_primitive_type;
        std::string entity_robot_primitive_type;
    };
    struct ROBOT_TO_ROBOT_DATA_V3 : BASE_DATA{
        std::vector<std::string> entity_one;
        std::vector<std::string> entity_two;
        std::string entity_one_primitive_type;
        std::string entity_two_primitive_type;
    };

    using DataV3 = std::variant<ENVIRONMENT_TO_ROBOT_DATA_V3, ROBOT_TO_ROBOT_DATA_V3>;

    // Complete content of a configuration file. The version is given by the variant alternative.
    struct DOCUMENT_V2{
        bool zero_indexed;
        std::vector<Data> vfi_array;
    };
    struct DOCUMENT_V3{
        bool zero_indexed;
        METADATA metadata;
        std::vector<ROBOT> robots;
        std::vector<ENVIRONMENT_ENTITY> environment_entities;
        std::vector<ROBOT_ENTITY> robot_entities;
        std::vector<DataV3> vfi_array;
        POSE_FORMAT pose_format = POSE_FORMAT::TRANSLATION_ROTATION; //Default value
    };

    using Document = std::variant<DOCUMENT_V2, DOCUMENT_V3>;

protected:
    VFIConfigurationFile() = default;

public:
    virtual ~VFIConfigurationFile() = default;

    /**
     * @brief load_data loads a configuration file.
     * @param config_file The name of the file including its path and format.
     */
    virtual void load_data(const std::string& config_file) = 0;

    /**
     * @brief get_data gets the vector that contains the VFI configurations.
     * @return The desired data vector.
     */
    virtual std::vector<Data>  get_data() const = 0;

    /**
     * @brief get_document gets the complete content of the loaded configuration file.
     * @return A DOCUMENT_V2 or a DOCUMENT_V3, depending on the file version.
     */
    virtual Document get_document() const = 0;

    /**
     * @brief get_vfi_file_version gets the configuration file version.
     * @return The desired file version.
     */
    virtual int get_vfi_file_version() const = 0;

    /**
     * @brief is_zero_indexed.
     * @return Returns true if the configuration file uses a zero-indexed convention to
     *         describe the joint and robot indexes. False otherwise.
     */
    virtual bool is_zero_indexed() const = 0;

    /**
     * @brief save_data saves a configuration file containing the VFI constraints.
     * @param data the vector that contains the VFI configurations
     * @param vfi_file_version The desired format version
     * @param zero_indexed To define if the data uses a zero-indexed convention.
     * @param config_file The desired name of the file including its path and format.
     */
    virtual void save_data(const std::vector<Data>& data,
                           const int& vfi_file_version,
                           const bool& zero_indexed,
                           const std::string& config_file) = 0;

};


}
