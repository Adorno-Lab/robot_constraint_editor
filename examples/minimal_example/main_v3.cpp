#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_v3.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics/DQ.h>
#include <cmath>
#include <iostream>
using namespace DQ_robotics;
using namespace DQ_robotics_extensions;



int main()
{
    // Load a version 3 file. The file is validated when it is loaded.
    auto ri = std::make_shared<VFIConfigurationFileYaml>();
    auto rce = RobotConstraintEditor(ri);
    rce.load_data("config_file_v3.yaml");

    const auto document = rce.get_document();
    std::cout << "Robot: " << document.robots.at(0).name
              << " (" << document.robots.at(0).dim_configuration << " joints)" << std::endl;
    for (const auto& entity : document.environment_entities)
        std::cout << "Environment entity: " << entity.name << std::endl;
    for (const auto& entity : document.robot_entities)
        std::cout << "Robot entity: " << entity.name << " (joint " << entity.joint_index << ")" << std::endl;
    std::cout << "Number of VFIs: " << document.vfi_array.size() << std::endl;


    // Add a robot entity attached to joint 4. Its offset is a unit dual quaternion
    // (rotation of 90 degrees about the z-axis and translation of 0.1 m along the z-axis).
    const DQ r = cos(M_PI/4.0) + k_*sin(M_PI/4.0);
    const DQ t = 0.1*k_;
    DQ_robotics_extensions::VFIConfigurationFile::ROBOT_ENTITY entity;
    entity.name = "elbow_sphere";
    entity.robot_index = 1;
    entity.joint_index = 4;
    entity.offset = VFIConfigurationFileV3::dq_to_pose(r + 0.5*E_*t*r);
    rce.add_robot_entity(entity);


    // Add a new constraint that uses the new entity---------
    DQ_robotics_extensions::VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_DATA_V3 data;
    data.vfi_type = "ENVIRONMENT_TO_ROBOT";
    data.entity_environment = {"Plane"};
    data.entity_robot = {"elbow_sphere"};
    data.entity_environment_primitive_type = "PLANE";
    data.entity_robot_primitive_type = "POINT";
    data.safe_distance = 0.1;
    data.buffer = 0.02;
    data.vfi_gain = 1.0;
    data.direction = "RESTRICTED_ZONE";
    data.tag = "C6";
    rce.add_data(data);


    //----Edit a constraint
    rce.edit_data("C3", "safe_distance", 0.08);

    //----Rename an entity. The constraints that use it are updated.
    rce.rename_entity("rsphere", "tool_sphere");


    // Save the document. It is validated before the file is written.
    rce.save_document("config_file_v3_2.yaml");


    // Invalid documents are detected. For instance, a VFI that uses an undefined entity:
    rce.edit_data("C6", "entity_robot", std::vector<std::string>{"undefined_entity"});
    try {
        rce.validate();
    } catch (const std::runtime_error& e) {
        std::cout << "Invalid document: " << e.what() << std::endl;
    }

    //------------------------------


    return 0;
}
