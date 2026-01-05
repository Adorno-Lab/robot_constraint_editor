
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/utils.hpp>
using namespace DQ_robotics_extensions;



int main()
{
    auto ri = VFIConfigurationFileYaml("config_file.yaml");
    VFIConfigurationFileRawData::show_data(ri.get_raw_data(), ri.get_vfi_file_version(), ri.get_zero_indexed_status());

    //-- Edit the YAML file-----

    auto rce = RobotConstraintEditor();
    rce.load_data("config_file.yaml");


    // Add a new constraint---------
    DQ_robotics_extensions::VFIConfigurationFile::ROBOT_TO_ROBOT_RAW_DATA data;
    data.vfi_type = "ROBOT_TO_ROBOT";
    data.cs_entity_one = {"entity1", "entity2"};
    data.cs_entity_two = {"entity3", "entity4"};
    data.entity_one_primitive_type = "LINE";
    data.entity_two_primitive_type = "PLANE";
    data.robot_index_one = 1;
    data.robot_index_two = 3;
    data.joint_index_one = 5;
    data.joint_index_two = 6;
    data.safe_distance = 2.3;
    data.vfi_gain = 5.1;
    data.direction = "RESTRICTED_ZONE";
    data.tag = "TAG_X1";
    rce.add_data(data);

    //----Edit a constraint
    rce.edit_data("C3", "tag", std::string("C33"));


    rce.save_data("config_file2.yaml", 2, false);



    //------------------------------

    //ri.show_raw_data(rce.get_raw_data(), ri.get_vfi_file_version(), ri.get_zero_indexed_status());

    return 0;
}
