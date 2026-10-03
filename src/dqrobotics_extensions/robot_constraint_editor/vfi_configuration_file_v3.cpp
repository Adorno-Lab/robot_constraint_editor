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

#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/utils.hpp>
#include <dqrobotics/DQ.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <variant>

namespace DQ_robotics_extensions
{
namespace VFIConfigurationFileV3
{

namespace
{

using DQ_robotics::DQ;
using POSE = VFIConfigurationFile::POSE;
using ROBOT = VFIConfigurationFile::ROBOT;
using ENVIRONMENT_ENTITY = VFIConfigurationFile::ENVIRONMENT_ENTITY;
using ROBOT_ENTITY = VFIConfigurationFile::ROBOT_ENTITY;
using ENVIRONMENT_TO_ROBOT_DATA_V3 = VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3;
using ROBOT_TO_ROBOT_DATA_V3 = VFIConfigurationFile::ROBOT_TO_ROBOT_DATA_V3;

const std::vector<std::string> PRIMITIVE_TYPES = {"POINT", "LINE", "PLANE", "LINESEGMENT", "LINE_ANGLE"};
const std::vector<std::string> DIRECTIONS = {"RESTRICTED_ZONE", "SAFE_ZONE"};
const std::vector<std::string> ATTACHED_DIRECTIONS = {"i_", "j_", "k_", "-i_", "-j_", "-k_"};

/**
 * @brief _to_string converts a double into a string without losing digits.
 */
std::string _to_string(const double& value)
{
    std::ostringstream stream;
    stream.precision(std::numeric_limits<double>::max_digits10);
    stream << value;
    return stream.str();
}

template<std::size_t N>
bool _is_finite(const std::array<double, N>& values)
{
    return std::all_of(values.begin(), values.end(), [](const double& value){ return std::isfinite(value); });
}

template<std::size_t N>
Eigen::VectorXd _to_vector(const std::array<double, N>& values)
{
    return Eigen::Map<const Eigen::VectorXd>(values.data(), N);
}

/**
 * @brief _check_value checks that a value is one of the allowed values.
 * @param value The value to check.
 * @param allowed The allowed values.
 * @param context The location of the value, used in the error message.
 */
void _check_value(const std::string& value, const std::vector<std::string>& allowed, const std::string& context)
{
    if (std::find(allowed.begin(), allowed.end(), value) == allowed.end())
        throw std::runtime_error(context + ": invalid value '" + value + "'. Possible values: "
                                 + join_vector(allowed) + ".");
}

/**
 * @brief _check_unit checks that x is a unit dual quaternion (i.e., is_unit(x), which uses DQ_threshold).
 *        The values of x must be finite: is_unit() does not detect NaN.
 * @param x The dual quaternion to check.
 * @param context The location of x, used in the error messages.
 */
void _check_unit(const DQ& x, const std::string& context)
{
    if (!DQ_robotics::is_unit(x))
    {
        const Eigen::Matrix<double, 8, 1> x_norm = DQ_robotics::vec8(DQ_robotics::norm(x));
        throw std::runtime_error(context + "not a unit dual quaternion (norm = " + _to_string(x_norm(0))
                                 + " + E_*(" + _to_string(x_norm(4)) + ")).");
    }
}

/**
 * @brief _check_pose checks that a POSE is a unit dual quaternion.
 */
void _check_pose(const POSE& pose, const std::string& context)
{
    if (!_is_finite(pose.translation) || !_is_finite(pose.rotation))
        throw std::runtime_error(context + ": contains non-finite values.");
    _check_unit(pose_to_dq(pose), context + ": ");
}

void _check_vfi_type(const std::string& vfi_type, const std::string& expected, const std::string& context)
{
    if (vfi_type != expected)
        throw std::runtime_error(context + ": vfi_type must be '" + expected + "', found '" + vfi_type + "'.");
}

/**
 * @brief _check_entity_list checks the primitive type of an entity list, the number of
 *        entities it requires, and that every entity is defined in the corresponding table.
 * @param entities The entity list.
 * @param primitive_type The primitive type of the entity list.
 * @param table The entity table in which the entities must be defined.
 * @param key The key of the entity list (e.g., "entity_robot"), used in the error messages.
 * @param table_name The name of the entity table, used in the error messages.
 * @param context The location of the VFI, used in the error messages.
 */
template<typename ENTITY>
void _check_entity_list(const std::vector<std::string>& entities,
                        const std::string& primitive_type,
                        const std::map<std::string, const ENTITY*>& table,
                        const std::string& key,
                        const std::string& table_name,
                        const std::string& context)
{
    _check_value(primitive_type, PRIMITIVE_TYPES, context + ", " + key + "_primitive_type");

    const std::size_t expected_size = (primitive_type == "LINESEGMENT") ? 3 : 1;
    if (entities.size() != expected_size)
        throw std::runtime_error(context + ", " + key + ": " + primitive_type + " requires "
                                 + std::to_string(expected_size) + " entities, found "
                                 + std::to_string(entities.size()) + ".");

    for (const auto& name : entities)
        if (table.find(name) == table.end())
            throw std::runtime_error(context + ", " + key + ": the entity '" + name
                                     + "' is not defined in " + table_name + ".");
}

/**
 * @brief _check_robot_linesegment checks that the three robot entities of a LINESEGMENT are
 *        attached to the same robot_index and joint_index. It must be called after _check_entity_list.
 */
void _check_robot_linesegment(const std::vector<std::string>& entities,
                              const std::string& primitive_type,
                              const std::map<std::string, const ROBOT_ENTITY*>& robot_entities,
                              const std::string& key,
                              const std::string& context)
{
    if (primitive_type != "LINESEGMENT")
        return;
    const ROBOT_ENTITY* line = robot_entities.at(entities.at(0));
    for (std::size_t i = 1; i < entities.size(); ++i)
    {
        const ROBOT_ENTITY* point = robot_entities.at(entities.at(i));
        if (point->robot_index != line->robot_index || point->joint_index != line->joint_index)
            throw std::runtime_error(context + ", " + key + ": the entities of a LINESEGMENT must have "
                                     "the same robot_index and joint_index.");
    }
}

} // namespace


DQ_robotics::DQ pose_to_dq(const VFIConfigurationFile::POSE& pose)
{
    const DQ r(_to_vector(pose.rotation));
    const DQ t(_to_vector(pose.translation));
    return r + 0.5*DQ_robotics::E_*t*r;
}


VFIConfigurationFile::POSE dq_to_pose(const DQ_robotics::DQ& x)
{
    const Eigen::Matrix<double, 8, 1> x_vec8 = DQ_robotics::vec8(x);
    if (!x_vec8.allFinite())
        throw std::runtime_error("the dual quaternion contains non-finite values.");
    _check_unit(x, "");

    const Eigen::Vector4d rotation = DQ_robotics::vec4(DQ_robotics::rotation(x));
    const Eigen::Vector3d translation = DQ_robotics::vec3(DQ_robotics::translation(x));

    POSE pose;
    pose.rotation = {rotation(0), rotation(1), rotation(2), rotation(3)};
    pose.translation = {translation(0), translation(1), translation(2)};
    return pose;
}


void validate(const VFIConfigurationFile::DOCUMENT_V3& document)
{
    const int first_index = document.zero_indexed ? 0 : 1;

    // Rule 2 (partially) and Section 4
    if (document.robots.size() != 1)
        throw std::runtime_error("robots: exactly one robot is required, found "
                                 + std::to_string(document.robots.size()) + ".");
    const ROBOT& robot = document.robots.front();
    if (robot.robot_index != first_index)
        throw std::runtime_error("robots[0]: robot_index must be " + std::to_string(first_index)
                                 + " when zero_indexed is " + bool2string(document.zero_indexed) + ".");
    if (robot.dim_configuration < 1)
        throw std::runtime_error("robots[0]: dim_configuration must be positive.");
    const int last_joint_index = first_index + robot.dim_configuration - 1;

    // Rules 3, 7, 8, and 9 for the entities
    std::set<std::string> names;
    std::map<std::string, const ENVIRONMENT_ENTITY*> environment_entities;
    std::map<std::string, const ROBOT_ENTITY*> robot_entities;

    auto check_unique_name = [&names](const std::string& name, const std::string& context)
    {
        if (!names.insert(name).second)
            throw std::runtime_error(context + ": the name '" + name + "' is already used by another entity.");
    };

    for (std::size_t i = 0; i < document.environment_entities.size(); ++i)
    {
        const ENVIRONMENT_ENTITY& entity = document.environment_entities.at(i);
        const std::string context = "environment_entities[" + std::to_string(i) + "] ('" + entity.name + "')";
        check_unique_name(entity.name, context);
        _check_pose(entity.pose, context + ", pose");
        _check_value(entity.attached_direction, ATTACHED_DIRECTIONS, context + ", attached_direction");
        environment_entities[entity.name] = &entity;
    }

    for (std::size_t i = 0; i < document.robot_entities.size(); ++i)
    {
        const ROBOT_ENTITY& entity = document.robot_entities.at(i);
        const std::string context = "robot_entities[" + std::to_string(i) + "] ('" + entity.name + "')";
        check_unique_name(entity.name, context);
        if (entity.robot_index != robot.robot_index)
            throw std::runtime_error(context + ": robot_index must match robots[0].robot_index ("
                                     + std::to_string(robot.robot_index) + ").");
        if (entity.joint_index < first_index || entity.joint_index > last_joint_index)
            throw std::runtime_error(context + ": joint_index must be in the range ["
                                     + std::to_string(first_index) + ", " + std::to_string(last_joint_index)
                                     + "], found " + std::to_string(entity.joint_index) + ".");
        _check_pose(entity.offset, context + ", offset");
        _check_value(entity.attached_direction, ATTACHED_DIRECTIONS, context + ", attached_direction");
        robot_entities[entity.name] = &entity;
    }

    // Rules 4, 5, 6, 9, and 10 for the VFIs
    std::set<std::string> tags;
    for (std::size_t i = 0; i < document.vfi_array.size(); ++i)
    {
        std::visit([&](const auto& data) {
            using T = std::decay_t<decltype(data)>;
            const std::string context = "vfi_array[" + std::to_string(i) + "] (tag '" + data.tag + "')";

            if (!tags.insert(data.tag).second)
                throw std::runtime_error(context + ": the tag is already used by another VFI.");
            _check_value(data.direction, DIRECTIONS, context + ", direction");

            if constexpr (std::is_same_v<T, ENVIRONMENT_TO_ROBOT_DATA_V3>) {
                _check_vfi_type(data.vfi_type, "ENVIRONMENT_TO_ROBOT", context);
                _check_entity_list(data.entity_environment, data.entity_environment_primitive_type,
                                   environment_entities, "entity_environment", "environment_entities", context);
                _check_entity_list(data.entity_robot, data.entity_robot_primitive_type,
                                   robot_entities, "entity_robot", "robot_entities", context);
                _check_robot_linesegment(data.entity_robot, data.entity_robot_primitive_type,
                                         robot_entities, "entity_robot", context);

            } else if constexpr (std::is_same_v<T, ROBOT_TO_ROBOT_DATA_V3>) {
                _check_vfi_type(data.vfi_type, "ROBOT_TO_ROBOT", context);
                _check_entity_list(data.entity_one, data.entity_one_primitive_type,
                                   robot_entities, "entity_one", "robot_entities", context);
                _check_entity_list(data.entity_two, data.entity_two_primitive_type,
                                   robot_entities, "entity_two", "robot_entities", context);
                _check_robot_linesegment(data.entity_one, data.entity_one_primitive_type,
                                         robot_entities, "entity_one", context);
                _check_robot_linesegment(data.entity_two, data.entity_two_primitive_type,
                                         robot_entities, "entity_two", context);
            }
        }, document.vfi_array.at(i));
    }
}

} // namespace VFIConfigurationFileV3
} // namespace DQ_robotics_extensions
