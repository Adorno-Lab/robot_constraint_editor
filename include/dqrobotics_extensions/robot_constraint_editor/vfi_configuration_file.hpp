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
#include <string>
#include <vector>
#include <variant>

namespace DQ_robotics_extensions
{

class VFIConfigurationFile
{
public:

    struct ENVIRONMENT_TO_ROBOT_RAW_DATA{
        std::string vfi_type;
        std::vector<std::string> cs_entity_environment;
        std::vector<std::string> cs_entity_robot;
        std::string entity_environment_primitive_type;
        std::string entity_robot_primitive_type;
        int robot_index;
        int joint_index;
        double safe_distance;
        double vfi_gain;
        std::string direction;
        std::string tag;
    };
    struct ROBOT_TO_ROBOT_RAW_DATA{
        std::string vfi_type;
        std::vector<std::string> cs_entity_one;
        std::vector<std::string> cs_entity_two;
        std::string entity_one_primitive_type;
        std::string entity_two_primitive_type;
        int robot_index_one;
        int robot_index_two;
        int joint_index_one;
        int joint_index_two;
        double safe_distance;
        double vfi_gain;
        std::string direction;
        std::string tag;
    };

    using RawData = std::variant<ENVIRONMENT_TO_ROBOT_RAW_DATA, ROBOT_TO_ROBOT_RAW_DATA>;

protected:
    VFIConfigurationFile() = default;

public:
    virtual ~VFIConfigurationFile() = default;

    /**
     * @brief get_raw_data gets the raw data vector from a YAML file.
     * @return A raw data vector.
     */
    virtual std::vector<RawData>  get_raw_data() = 0;

    /**
     * @brief get_vfi_file_version gets the vfi_file_version data from
     *              the YAML file.
     * @return The desired data.
     */
    virtual int get_vfi_file_version() = 0;

    /**
     * @brief get_zero_indexed_status gets the zero_indexed data from
     *          the YAML file.
     * @return The desired data.
     */
    virtual bool get_zero_indexed_status() = 0;


};


}
