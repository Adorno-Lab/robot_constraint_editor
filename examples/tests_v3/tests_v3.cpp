#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <dqrobotics/DQ.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace DQ_robotics_extensions;
using namespace DQ_robotics;

using File = VFIConfigurationFile;

namespace
{

// Valid version 3 file used as the base of the test cases
const std::string BASE_FILE = R"(vfi_file_version: 3
zero_indexed: false

robots:
  -
    robot_index: 1
    name: "R"
    dim_configuration: 7

environment_entities:
  -
    name: "Plane"
    pose:
      translation: [0.0, 0.0, 0.05]
      rotation:    [1.0, 0.0, 0.0, 0.0]

robot_entities:
  -
    name: "line"
    robot_index: 1
    joint_index: 7
    offset:
      translation: [0.0, 0.0, 0.0]
      rotation:    [1.0, 0.0, 0.0, 0.0]
  -
    name: "p1"
    robot_index: 1
    joint_index: 7
    offset:
      translation: [0.0, 0.0, -0.05]
      rotation:    [1.0, 0.0, 0.0, 0.0]
  -
    name: "p2"
    robot_index: 1
    joint_index: 7
    offset:
      translation: [0.0, 0.0, 0.10]
      rotation:    [1.0, 0.0, 0.0, 0.0]

vfi_array:
  -
    vfi_type: "ENVIRONMENT_TO_ROBOT"
    entity_environment: ["Plane"]
    entity_robot: ["line", "p1", "p2"]
    entity_environment_primitive_type: "PLANE"
    entity_robot_primitive_type: "LINESEGMENT"
    safe_distance: 0.05
    vfi_gain: 1.0
    direction: "RESTRICTED_ZONE"
    tag: "C1"
  -
    vfi_type: "ROBOT_TO_ROBOT"
    entity_one: ["p1"]
    entity_two: ["p2"]
    entity_one_primitive_type: "POINT"
    entity_two_primitive_type: "POINT"
    safe_distance: 0.1
    vfi_gain: 1.0
    direction: "RESTRICTED_ZONE"
    tag: "C2"
)";

const std::string TMP_FILE = "tests_v3_tmp.yaml";
const std::string SAVED_FILE = "tests_v3_saved.yaml";
const std::string SAVED_FILE_2 = "tests_v3_saved_2.yaml";

int failures = 0;

void check(const bool& condition, const std::string& description)
{
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << description << std::endl;
    if (!condition)
        failures++;
}

bool near(const double& a, const double& b, const double& tolerance = 1e-12)
{
    return std::abs(a - b) <= tolerance;
}

void write_file(const std::string& content)
{
    std::ofstream file(TMP_FILE);
    file << content;
}

std::string read_file(const std::string& name)
{
    std::ifstream file(name);
    std::stringstream content;
    content << file.rdbuf();
    return content.str();
}

bool contains(const std::string& text, const std::string& expected)
{
    return text.find(expected) != std::string::npos;
}

/**
 * @brief save_and_reload saves a document, loads it again, and returns the loaded document.
 *        It also checks that saving the loaded document gives the same file (no data is lost).
 */
File::DOCUMENT_V3 save_and_reload(const File::DOCUMENT_V3& document, const std::string& description)
{
    auto file = std::make_shared<VFIConfigurationFileYaml>();
    file->save_document(document, SAVED_FILE);
    file->load_data(SAVED_FILE);
    const auto reloaded = std::get<File::DOCUMENT_V3>(file->get_document());
    file->save_document(reloaded, SAVED_FILE_2);
    check(read_file(SAVED_FILE) == read_file(SAVED_FILE_2), description + ": save -> load -> save gives the same file");
    return reloaded;
}

/**
 * @brief replace returns a copy of text where 'from' (which must appear in text) is replaced by 'to'.
 */
std::string replace(std::string text, const std::string& from, const std::string& to)
{
    const auto position = text.find(from);
    if (position == std::string::npos)
        throw std::runtime_error("Test setup error: '" + from + "' not found.");
    return text.replace(position, from.size(), to);
}

/**
 * @brief check_rejected checks that loading content throws an exception whose message contains expected_message.
 */
void check_rejected(const std::string& description, const std::string& content, const std::string& expected_message)
{
    write_file(content);
    auto file = std::make_shared<VFIConfigurationFileYaml>();
    try {
        file->load_data(TMP_FILE);
        check(false, "Rejects: " + description + " (the file was accepted)");
    } catch (const std::runtime_error& e) {
        const std::string message = e.what();
        const bool ok = message.find(expected_message) != std::string::npos;
        check(ok, "Rejects: " + description + "\n         -> " + message);
    }
}

void test_example_file()
{
    std::cout << "\n--- config_file_v3.yaml ---" << std::endl;
    auto file = std::make_shared<VFIConfigurationFileYaml>();
    file->load_data("config_file_v3.yaml");
    const File::Document document = file->get_document();

    check(file->get_vfi_file_version() == 3, "vfi_file_version is 3");
    check(!file->is_zero_indexed(), "zero_indexed is false");
    check(std::holds_alternative<File::DOCUMENT_V3>(document), "get_document() returns a DOCUMENT_V3");

    const auto& doc = std::get<File::DOCUMENT_V3>(document);
    check(doc.metadata.source == "panda_example.ttt", "metadata.source");
    check(doc.robots.size() == 1 && doc.robots.at(0).name == "Franka" && doc.robots.at(0).dim_configuration == 7, "robots");
    check(doc.environment_entities.size() == 4, "4 environment entities");
    check(doc.robot_entities.size() == 3, "3 robot entities");
    check(doc.vfi_array.size() == 5, "5 VFIs");
    check(doc.pose_format == File::POSE_FORMAT::TRANSLATION_ROTATION, "mixed pose forms -> TRANSLATION_ROTATION");

    const auto& plane = doc.environment_entities.at(1);
    check(plane.name == "Plane" && near(plane.pose.translation.at(2), 0.05) && plane.attached_direction == "k_", "Plane pose");
    const auto& cylinder = doc.environment_entities.at(2);
    check(cylinder.attached_direction == "k_", "Cylinder uses the default attached_direction");

    const auto& sphere = doc.environment_entities.at(3);
    check(sphere.name == "obs_sphere"
              && near(sphere.pose.translation.at(0), 0.4) && near(sphere.pose.translation.at(1), 0.25)
              && near(sphere.pose.translation.at(2), 0.45) && near(sphere.pose.rotation.at(0), 1.0),
          "obs_sphere (8 coefficients) -> translation [0.4, 0.25, 0.45]");

    const auto& rsphere = doc.robot_entities.at(1);
    check(rsphere.name == "rsphere" && rsphere.robot_index == 1 && rsphere.joint_index == 7
              && near(rsphere.offset.translation.at(2), 0.05), "rsphere offset");

    const auto& c1 = std::get<File::ENVIRONMENT_TO_ROBOT_DATA_V3>(doc.vfi_array.at(0));
    check(c1.tag == "C1" && c1.entity_environment == std::vector<std::string>{"x_inertial"}
              && c1.entity_robot == std::vector<std::string>{"rline"}
              && c1.entity_robot_primitive_type == "LINE_ANGLE" && near(c1.safe_distance, 5.0)
              && c1.direction == "SAFE_ZONE", "C1 (ENVIRONMENT_TO_ROBOT)");
    const auto& c2 = std::get<File::ROBOT_TO_ROBOT_DATA_V3>(doc.vfi_array.at(1));
    check(c2.tag == "C2" && c2.entity_one == std::vector<std::string>{"r_base_sphere"}
              && c2.entity_two == std::vector<std::string>{"rsphere"} && near(c2.safe_distance, 0.3),
          "C2 (ROBOT_TO_ROBOT)");

    bool get_data_throws = false;
    try { file->get_data(); } catch (const std::runtime_error&) { get_data_throws = true; }
    check(get_data_throws, "get_data() throws for a version 3 file");
}

void test_base_file()
{
    std::cout << "\n--- Base file ---" << std::endl;
    write_file(BASE_FILE);
    auto file = std::make_shared<VFIConfigurationFileYaml>();
    file->load_data(TMP_FILE);
    const auto doc = std::get<File::DOCUMENT_V3>(file->get_document());
    check(!doc.metadata.description.size() && doc.environment_entities.size() == 1, "Loads without metadata");
    const auto& c1 = std::get<File::ENVIRONMENT_TO_ROBOT_DATA_V3>(doc.vfi_array.at(0));
    check(c1.buffer == 0.0, "buffer defaults to 0.0");
    check(c1.entity_robot.size() == 3, "LINESEGMENT with 3 entities");
}

void test_unit_dual_quaternion_form()
{
    std::cout << "\n--- Unit dual quaternion form ---" << std::endl;
    // Plane: rotation of 90 deg about z, translation [1, 2, 3]
    const DQ r = cos(M_PI/4.0) + k_*sin(M_PI/4.0);
    const DQ t = 1.0*i_ + 2.0*j_ + 3.0*k_;
    const auto x = vec8(r + 0.5*E_*t*r);
    std::ostringstream plane;
    plane.precision(17);
    plane << "pose: [";
    for (int i = 0; i < 8; ++i)
        plane << x(i) << (i < 7 ? ", " : "]");

    std::string content = replace(BASE_FILE,
                                  "pose:\n      translation: [0.0, 0.0, 0.05]\n      rotation:    [1.0, 0.0, 0.0, 0.0]",
                                  plane.str());
    content = replace(content, "offset:\n      translation: [0.0, 0.0, 0.0]\n      rotation:    [1.0, 0.0, 0.0, 0.0]",
                      "offset: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]");
    content = replace(content, "offset:\n      translation: [0.0, 0.0, -0.05]\n      rotation:    [1.0, 0.0, 0.0, 0.0]",
                      "offset: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.025]");
    content = replace(content, "offset:\n      translation: [0.0, 0.0, 0.10]\n      rotation:    [1.0, 0.0, 0.0, 0.0]",
                      "offset: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]");
    write_file(content);

    auto file = std::make_shared<VFIConfigurationFileYaml>();
    file->load_data(TMP_FILE);
    const auto doc = std::get<File::DOCUMENT_V3>(file->get_document());
    const auto& pose = doc.environment_entities.at(0).pose;
    check(near(pose.translation.at(0), 1.0) && near(pose.translation.at(1), 2.0) && near(pose.translation.at(2), 3.0),
          "Rotated pose -> translation [1, 2, 3]");
    check(near(pose.rotation.at(0), cos(M_PI/4.0)) && near(pose.rotation.at(3), sin(M_PI/4.0)), "Rotated pose -> rotation");
    check(near(doc.robot_entities.at(2).offset.translation.at(2), 0.10), "p2 offset -> translation [0, 0, 0.10]");
    check(doc.pose_format == File::POSE_FORMAT::UNIT_DUAL_QUATERNION, "Only 8 coefficients -> UNIT_DUAL_QUATERNION");

    const auto reloaded = save_and_reload(doc, "Rotated pose (8 coefficients)");
    const auto& reloaded_pose = reloaded.environment_entities.at(0).pose;
    check(reloaded.pose_format == File::POSE_FORMAT::UNIT_DUAL_QUATERNION, "Saved with 8 coefficients");
    check(near(reloaded_pose.translation.at(0), 1.0) && near(reloaded_pose.translation.at(1), 2.0)
              && near(reloaded_pose.translation.at(2), 3.0) && near(reloaded_pose.rotation.at(0), cos(M_PI/4.0))
              && near(reloaded_pose.rotation.at(3), sin(M_PI/4.0)), "Rotated pose after save -> load");
}

void test_invalid_files()
{
    std::cout << "\n--- Invalid files ---" << std::endl;
    const std::string p1_block = "name: \"p1\"\n    robot_index: 1\n    joint_index: 7";
    const std::string p2_block = "name: \"p2\"\n    robot_index: 1\n    joint_index: 7";
    const std::string plane_rotation = "translation: [0.0, 0.0, 0.05]\n      rotation:    [1.0, 0.0, 0.0, 0.0]";
    const std::string plane_pose = "pose:\n      " + plane_rotation;
    const std::string robot_block = "  -\n    robot_index: 1\n    name: \"R\"\n    dim_configuration: 7\n";

    check_rejected("missing zero_indexed", replace(BASE_FILE, "zero_indexed: false\n", ""),
                   "missing required key 'zero_indexed'");
    check_rejected("two robots", replace(BASE_FILE, robot_block, robot_block + robot_block),
                   "exactly one robot is required");
    check_rejected("robot_index 0 with zero_indexed false", replace(BASE_FILE, "    robot_index: 1\n    name: \"R\"",
                                                                    "    robot_index: 0\n    name: \"R\""),
                   "robot_index must be 1");
    check_rejected("missing robot_entities", replace(BASE_FILE, "robot_entities:", "robot_entities_typo:"),
                   "missing required key 'robot_entities'");
    check_rejected("duplicated entity name", replace(BASE_FILE, "name: \"p2\"", "name: \"Plane\""),
                   "already used by another entity");
    check_rejected("robot entity with another robot_index",
                   replace(BASE_FILE, p1_block, "name: \"p1\"\n    robot_index: 2\n    joint_index: 7"),
                   "must match robots[0].robot_index");
    check_rejected("joint_index out of range",
                   replace(BASE_FILE, p1_block, "name: \"p1\"\n    robot_index: 1\n    joint_index: 8"),
                   "joint_index must be in the range [1, 7]");
    check_rejected("non-unit rotation",
                   replace(BASE_FILE, plane_rotation, "translation: [0.0, 0.0, 0.05]\n      rotation:    [1.0, 0.1, 0.0, 0.0]"),
                   "not a unit dual quaternion");
    check_rejected("rotation with 10 digits (not unit within DQ_threshold)",
                   replace(BASE_FILE, plane_rotation, "translation: [0.0, 0.0, 0.05]\n      rotation:    [0.7071067812, 0.0, 0.0, 0.7071067812]"),
                   "not a unit dual quaternion");
    check_rejected("rotation with 3 numbers",
                   replace(BASE_FILE, plane_rotation, "translation: [0.0, 0.0, 0.05]\n      rotation:    [1.0, 0.0, 0.0]"),
                   "must be a list of 4 numbers");
    check_rejected("8 coefficients with non-orthogonal parts",
                   replace(BASE_FILE, plane_pose, "pose: [1.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]"),
                   "not a unit dual quaternion (norm = 1 + E_*(0.10000000000000001))");
    check_rejected("8 coefficients with primary norm 1 + 5e-11",
                   replace(BASE_FILE, plane_pose, "pose: [1.00000000005, 0.0, 0.0, 0.0, 0.0, 0.2, 0.125, 0.225]"),
                   "not a unit dual quaternion");
    check_rejected("8 coefficients with non-unit primary part",
                   replace(BASE_FILE, plane_pose, "pose: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
                   "not a unit dual quaternion (norm = 2 + E_*(0))");
    check_rejected("7 coefficients",
                   replace(BASE_FILE, plane_pose, "pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
                   "must be a list of 8 numbers");
    check_rejected("invalid attached_direction",
                   replace(BASE_FILE, plane_pose, plane_pose + "\n    attached_direction: \"z_\""),
                   "attached_direction: invalid value 'z_'");
    check_rejected("unknown entity", replace(BASE_FILE, "entity_two: [\"p2\"]", "entity_two: [\"p3\"]"),
                   "'p3' is not defined in robot_entities");
    check_rejected("environment entity used as robot entity",
                   replace(BASE_FILE, "entity_one: [\"p1\"]", "entity_one: [\"Plane\"]"),
                   "'Plane' is not defined in robot_entities");
    check_rejected("LINESEGMENT with 1 entity",
                   replace(BASE_FILE, "entity_robot: [\"line\", \"p1\", \"p2\"]", "entity_robot: [\"line\"]"),
                   "LINESEGMENT requires 3 entities, found 1");
    check_rejected("POINT with 2 entities",
                   replace(BASE_FILE, "entity_one: [\"p1\"]", "entity_one: [\"p1\", \"p2\"]"),
                   "POINT requires 1 entities, found 2");
    check_rejected("LINESEGMENT on different joints",
                   replace(BASE_FILE, p2_block, "name: \"p2\"\n    robot_index: 1\n    joint_index: 6"),
                   "the same robot_index and joint_index");
    check_rejected("LINEANGLE primitive type",
                   replace(BASE_FILE, "entity_one_primitive_type: \"POINT\"", "entity_one_primitive_type: \"LINEANGLE\""),
                   "invalid value 'LINEANGLE'");
    check_rejected("invalid direction",
                   replace(BASE_FILE, "direction: \"RESTRICTED_ZONE\"\n    tag: \"C2\"",
                           "direction: \"FORBIDDEN_ZONE\"\n    tag: \"C2\""),
                   "invalid value 'FORBIDDEN_ZONE'");
    check_rejected("duplicated tag", replace(BASE_FILE, "tag: \"C2\"", "tag: \"C1\""),
                   "the tag is already used");
    check_rejected("unknown vfi_type", replace(BASE_FILE, "vfi_type: \"ROBOT_TO_ROBOT\"", "vfi_type: \"ROBOT_TO_SKY\""),
                   "unknown vfi_type 'ROBOT_TO_SKY'");
    check_rejected("V2 key cs_entity_one", replace(BASE_FILE, "entity_one: [\"p1\"]", "cs_entity_one: [\"p1\"]"),
                   "missing required key 'entity_one'");
    check_rejected("invalid buffer", replace(BASE_FILE, "    safe_distance: 0.1\n", "    safe_distance: 0.1\n    buffer: \"abc\"\n"),
                   "invalid value for 'buffer'");
}

void test_v2_file()
{
    std::cout << "\n--- Version 2 file ---" << std::endl;
    auto file = std::make_shared<VFIConfigurationFileYaml>();

    bool get_document_throws = false;
    try { file->get_document(); } catch (const std::runtime_error&) { get_document_throws = true; }
    check(get_document_throws, "get_document() throws before loading a file");

    file->load_data("config_file_v3.yaml");
    file->load_data("config_file.yaml");
    const File::Document document = file->get_document();
    check(file->get_vfi_file_version() == 2, "Loading a V2 file after a V3 file -> version 2");
    check(std::holds_alternative<File::DOCUMENT_V2>(document), "get_document() returns a DOCUMENT_V2");
    const auto& doc = std::get<File::DOCUMENT_V2>(document);
    check(doc.vfi_array.size() == file->get_data().size() && !doc.vfi_array.empty(),
          "DOCUMENT_V2 contains the same VFIs as get_data()");
    check(doc.zero_indexed == file->is_zero_indexed(), "DOCUMENT_V2 zero_indexed");
}

void test_save_v3()
{
    std::cout << "\n--- Save version 3 ---" << std::endl;
    auto file = std::make_shared<VFIConfigurationFileYaml>();
    file->load_data("config_file_v3.yaml");
    const auto original = std::get<File::DOCUMENT_V3>(file->get_document());

    // Translation and rotation form
    const auto reloaded = save_and_reload(original, "Example file");
    const std::string text = read_file(SAVED_FILE);
    check(reloaded.pose_format == File::POSE_FORMAT::TRANSLATION_ROTATION, "Saved with translation and rotation");
    check(reloaded.environment_entities.at(3).pose.translation == original.environment_entities.at(3).pose.translation
              && reloaded.robot_entities.at(2).offset.translation == original.robot_entities.at(2).offset.translation,
          "Poses are recovered exactly");
    check(contains(text, "vfi_file_version: 3\nzero_indexed: false\n\nmetadata:\n  description: "), "Layout: header and metadata");
    check(contains(text, "\nrobots:\n  -\n    robot_index: 1\n    name: \"Franka\"\n    dim_configuration: 7\n"), "Layout: robots");
    check(contains(text, "  -\n    name: \"Cylinder\"\n    pose:\n      translation: [0.45, -0.2, 0.3]\n"
                         "      rotation:    [1.0, 0.0, 0.0, 0.0]\n    attached_direction: \"k_\"\n"),
          "Layout: environment entity (default attached_direction is written)");
    check(contains(text, "    joint_index: 1\n    offset:\n      translation: [0.0, 0.0, -0.1]\n"), "Layout: robot entity");
    check(contains(text, "\nvfi_array:\n  -\n    vfi_type: \"ENVIRONMENT_TO_ROBOT\"\n    entity_environment: [\"x_inertial\"]\n"),
          "Layout: vfi_array");
    check(contains(text, "    safe_distance: 5.0\n    buffer: 0.0\n    vfi_gain: 1.0\n"), "Numbers: integral values with .0");
    check(contains(text, "    safe_distance: 0.05\n"), "Numbers: shortest exact representation");

    // Unit dual quaternion form
    auto document = original;
    document.pose_format = File::POSE_FORMAT::UNIT_DUAL_QUATERNION;
    const auto reloaded_dq = save_and_reload(document, "Example file (8 coefficients)");
    const std::string text_dq = read_file(SAVED_FILE);
    check(contains(text_dq, "    pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.125, 0.225]\n") && !contains(text_dq, "translation:"),
          "Layout: 8 coefficients");
    check(reloaded_dq.pose_format == File::POSE_FORMAT::UNIT_DUAL_QUATERNION
              && near(reloaded_dq.environment_entities.at(2).pose.translation.at(0), 0.45)
              && near(reloaded_dq.robot_entities.at(2).offset.translation.at(2), -0.1),
          "Poses after save -> load (8 coefficients)");

    // Strings with quotes, backslashes, and new lines
    document = original;
    document.metadata.description = "a \"quoted\" \\ text\nsecond line";
    document.robots.at(0).name = "Fr\"anka";
    std::get<File::ENVIRONMENT_TO_ROBOT_DATA_V3>(document.vfi_array.at(0)).tag = "C\"1";
    const auto reloaded_strings = save_and_reload(document, "Special characters");
    check(reloaded_strings.metadata.description == document.metadata.description
              && reloaded_strings.robots.at(0).name == "Fr\"anka"
              && std::get<File::ENVIRONMENT_TO_ROBOT_DATA_V3>(reloaded_strings.vfi_array.at(0)).tag == "C\"1",
          "Special characters are recovered");

    // Optional sections are omitted
    document = original;
    document.metadata = File::METADATA{};
    document.environment_entities.clear();
    document.vfi_array = {original.vfi_array.at(1)}; // ROBOT_TO_ROBOT only
    const auto reloaded_optional = save_and_reload(document, "Without optional sections");
    const std::string text_optional = read_file(SAVED_FILE);
    check(!contains(text_optional, "metadata:") && !contains(text_optional, "environment_entities:")
              && reloaded_optional.environment_entities.empty() && reloaded_optional.vfi_array.size() == 1,
          "metadata and environment_entities are omitted when empty");

    // An invalid document is rejected without modifying the file
    write_file("original content");
    document = original;
    std::get<File::ROBOT_TO_ROBOT_DATA_V3>(document.vfi_array.at(1)).tag = "C1";
    bool rejected = false;
    try { file->save_document(document, TMP_FILE); } catch (const std::runtime_error& e) {
        rejected = contains(e.what(), "the tag is already used");
    }
    check(rejected && read_file(TMP_FILE) == "original content", "Invalid document is rejected and the file is not modified");

    bool empty_path_rejected = false;
    try { file->save_document(original, ""); } catch (const std::runtime_error&) { empty_path_rejected = true; }
    check(empty_path_rejected, "Empty path is rejected");
}

void test_save_v2()
{
    std::cout << "\n--- Save version 2 ---" << std::endl;
    auto file = std::make_shared<VFIConfigurationFileYaml>();
    file->load_data("config_file.yaml");
    file->save_document(file->get_document(), SAVED_FILE);
    file->save_data(file->get_data(), 2, file->is_zero_indexed(), SAVED_FILE_2);
    check(read_file(SAVED_FILE) == read_file(SAVED_FILE_2), "save_document(DOCUMENT_V2) is equivalent to save_data()");
}

} // namespace

int main()
{
    try {
        test_example_file();
        test_base_file();
        test_unit_dual_quaternion_form();
        test_invalid_files();
        test_v2_file();
        test_save_v3();
        test_save_v2();
    } catch (const std::exception& e) {
        std::cout << "[FAIL] Unexpected exception: " << e.what() << std::endl;
        failures++;
    }
    for (const auto& name : {TMP_FILE, SAVED_FILE, SAVED_FILE_2})
        std::remove(name.c_str());

    std::cout << "\n" << (failures == 0 ? "All tests passed." : std::to_string(failures) + " test(s) failed.")
              << std::endl;
    return failures == 0 ? 0 : 1;
}
