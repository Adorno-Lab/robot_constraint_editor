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
#include <dqrobotics/DQ.h>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file.hpp>

namespace DQ_robotics_extensions
{

// Format-independent tools for version 3 configuration files.
// See design/specs_document/config_file_specification_v3.md
namespace VFIConfigurationFileV3 {

    /**
     * @brief pose_to_dq returns the unit dual quaternion x = r + 0.5*E_*t*r described by a POSE.
     * @param pose The POSE that contains vec3(t) and vec4(r).
     * @return The dual quaternion x. It is not normalized.
     */
    DQ_robotics::DQ pose_to_dq(const VFIConfigurationFile::POSE& pose);

    /**
     * @brief dq_to_pose extracts the rotation and translation of a unit dual quaternion.
     * @param x The unit dual quaternion (e.g., DQ(vec8) with the coefficients read from the file).
     * @return The POSE that contains vec3(translation(x)) and vec4(rotation(x)).
     * @throws std::runtime_error if x has non-finite values or is not a unit dual quaternion (is_unit(x)).
     */
    VFIConfigurationFile::POSE dq_to_pose(const DQ_robotics::DQ& x);

    /**
     * @brief validate checks the validation rules of Section 9. Rule 2 is checked only partially:
     *        the match between dim_configuration and the DQ_Kinematics model is checked by the RCM.
     * @param document The document to check.
     * @throws std::runtime_error describing the first rule that is not met.
     */
    void validate(const VFIConfigurationFile::DOCUMENT_V3& document);
}

}
