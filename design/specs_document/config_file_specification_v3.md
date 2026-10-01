# Configuration YAML File Specification: Version 3

Author: Juan Jose Quiroz-Omana

## 1. Introduction

Version 2 (V2) describes each VFI with CoppeliaSim object names (`cs_entity_*`).
When the configuration is loaded, the robot_constraint_manager (RCM) queries
CoppeliaSim to obtain:

- the pose of each environment primitive (`get_object_pose`), and
- the offset of each robot primitive with respect to its joint frame
  (`fkm(q, joint_index).conj() * get_object_pose(name)`).

Moreover, the attached direction of lines and planes is hard-coded to `k_`.

Version 3 (V3) stores this geometric data explicitly in the file. Therefore, the
RCM requires only a `DQ_Kinematics` model and the configuration file; CoppeliaSim
is no longer needed to build the constraints.

The configuration file is composed of the following elements:

| Element | Required | Description |
|---|---|---|
| `vfi_file_version` | yes | Must be `3`. |
| `zero_indexed` | yes | Index convention for `robot_index` and `joint_index`. |
| `metadata` | no | Informational data. Ignored by the RCM. |
| `robots` | yes | The robot whose kinematic model is used by the constraints. |
| `environment_entities` | no* | Named frames fixed in the workspace. |
| `robot_entities` | yes | Named frames kinematically attached to a robot joint. |
| `vfi_array` | yes | The VFI constraints. |

\* Required if at least one VFI of type `"ENVIRONMENT_TO_ROBOT"` is defined.

A complete example is provided in [config_file_v3.yaml](config_file_v3.yaml).

## 2. File structure

```yaml
vfi_file_version: 3
zero_indexed: false

metadata:
  description: "..."
  generated_by: "robot_constraint_editor"
  source: "..."

robots:
  - robot_index: 1
    name: "..."
    dim_configuration: 7

environment_entities:
  - name: "..."
    pose:
      translation: [x, y, z]
      rotation:    [w, x, y, z]
    attached_direction: "k_"

robot_entities:
  - name: "..."
    robot_index: 1
    joint_index: 7
    offset:
      translation: [x, y, z]
      rotation:    [w, x, y, z]
    attached_direction: "k_"

vfi_array:
  - vfi_type: "ENVIRONMENT_TO_ROBOT"
    ...
  - vfi_type: "ROBOT_TO_ROBOT"
    ...
```

### 2.1 Writing style

- Mappings (`pose`, `offset`, entities, VFIs) are written in YAML block style.
- Flow style (`[...]`) is used only for short lists of values: `translation`,
  `rotation`, the 8-element form of `pose`/`offset`, and the entity lists of the
  `vfi_array`.
- Writers must use 17 significant digits for the coefficients of `pose`/`offset`
  (i.e., `std::numeric_limits<double>::max_digits10`), so that the values are
  recovered exactly after a save/load cycle.

## 3. Header

| Parameter | Description/Possible values | Type | Required |
|---|---|---|---|
| `vfi_file_version` | `3` | int | yes |
| `zero_indexed` | `true`: the minimum `robot_index`/`joint_index` is 0. `false`: it is 1. | bool | yes |

### 3.1 `metadata`

Optional. Readers must ignore it.

| Parameter | Description | Type |
|---|---|---|
| `description` | Free text. | string |
| `generated_by` | Tool that generated the file (e.g., `"robot_constraint_editor"`). | string |
| `source` | Origin of the geometric data (e.g., a CoppeliaSim scene). | string |

## 4. `robots`

V3 supports a single robot. `robots` is a list to keep the structure compatible
with future multi-robot versions, but it must contain exactly one element.

| Parameter | Description | Type | Required |
|---|---|---|---|
| `robot_index` | Reserved. Must be `1` if `zero_indexed: false`, or `0` otherwise. | int | yes |
| `name` | Informational name of the robot. | string | yes |
| `dim_configuration` | Must match `DQ_Kinematics::get_dim_configuration_space()` of the model given to the RCM. | int | yes |

## 5. Geometric data

### 5.1 Pose (`pose` and `offset`)

Both `pose` and `offset` describe a unit dual quaternion `x`. They can be written
in either of the two forms below. The reader identifies the form from the YAML
node type (mapping or sequence), so no additional field is required. Each
`pose`/`offset` is parsed independently; therefore, both forms can be used in the
same file.

The two keys have different names because they are expressed in different frames
(see Sections 6 and 7).

#### 5.1.1 Translation and rotation (mapping)

```yaml
pose:
  translation: [x, y, z]
  rotation:    [w, x, y, z]
```

| Parameter | Description | Type |
|---|---|---|
| `translation` | Coefficients `[x, y, z]` of the pure quaternion `t = x*i_ + y*j_ + z*k_`, i.e., `vec3(t)`. Meters. | list of 3 doubles |
| `rotation` | Coefficients `[w, x, y, z]` of the unit quaternion `r = w + x*i_ + y*j_ + z*k_`, i.e., `vec4(r)`. | list of 4 doubles |

The corresponding unit dual quaternion is

```
x = r + 0.5*E_*t*r
```

#### 5.1.2 Unit dual quaternion (sequence)

```yaml
pose: [c1, c2, c3, c4, c5, c6, c7, c8]
```

The 8 coefficients of `x`, i.e., `vec8(x)`, in the order used by `DQ::vec8()` and
the `DQ(VectorXd)` constructor:

```
x = (c1 + c2*i_ + c3*j_ + c4*k_) + E_*(c5 + c6*i_ + c7*j_ + c8*k_)
```

where `c1`–`c4` are the primary part `P(x)` and `c5`–`c8` the dual part `D(x)`. The
sequence must contain exactly 8 numbers.

This form is convenient to paste values obtained in code (e.g., `vec8(x)`), but it
is not recommended for manual editing: since the dual part combines translation
and rotation, modifying a single coefficient breaks the unit condition.

#### 5.1.3 Normalization

Each `pose`/`offset` must be a unit dual quaternion within a tolerance of `1e-10`.
The reader throws an exception if the tolerance is exceeded; otherwise, it
normalizes the value (`x.normalize()`). Normalization is required because the
DQ Robotics library uses a threshold of `1e-12` (`DQ_threshold`), and some
methods used by the RCM (e.g., `DQ::translation()`) throw an exception for
non-unit dual quaternions.

#### 5.1.4 Writing

The robot_constraint_editor writes the mapping form (Section 5.1.1) by default,
and may provide an option to write the sequence form (Section 5.1.2). A single
form is used in the whole file.

### 5.2 `attached_direction`

The attached direction defines the line orientation (`LINE`, `LINE_ANGLE`) or the
plane normal (`PLANE`), expressed in the entity frame. Given the entity rotation
`r`, the direction in the reference frame is `r*d*r.conj()`.

| Possible values | Default |
|---|---|
| `"i_"`, `"j_"`, `"k_"`, `"-i_"`, `"-j_"`, `"-k_"` | `"k_"` |

Any other direction is obtained by rotating the entity frame. The sign is useful
for `PLANE` primitives, in which the normal defines the side of the plane that
the VFI refers to. The parameter is ignored for `POINT` primitives.

## 6. `environment_entities`

Frames fixed in the workspace. The `pose` is expressed in the same frame as the
output of `DQ_Kinematics::fkm()` (i.e., the frame in which the robot reference
frame is defined).

| Parameter | Description | Type | Required |
|---|---|---|---|
| `name` | Unique name (see Section 9). | string | yes |
| `pose` | Pose of the entity (Section 5.1). | pose | yes |
| `attached_direction` | Section 5.2. | string | no |

The pose is the initial value. It can be modified at runtime with
`RobotConstraintManager::update_vfi_workspace_pose()`.

## 7. `robot_entities`

Frames kinematically attached to a robot joint. The `offset` is the pose of the
entity with respect to the frame returned by `robot->fkm(q, j)`, where `robot` is
the `DQ_Kinematics` model given to the RCM and `j` is `joint_index` converted to
zero-indexed. That is, the pose of the entity is

```
x_entity = robot->fkm(q, j) * offset
```

| Parameter | Description | Type | Required |
|---|---|---|---|
| `name` | Unique name (see Section 9). | string | yes |
| `robot_index` | Reserved. Must match `robots[0].robot_index`. | int | yes |
| `joint_index` | Joint to which the entity is attached. Range: `[1, n]` if `zero_indexed: false`, `[0, n-1]` otherwise, where `n` is `dim_configuration`. | int | yes |
| `offset` | Offset with respect to the joint frame (Section 5.1). | pose | yes |
| `attached_direction` | Section 5.2. | string | no |

**Note:** the offset depends on the kinematic model. For instance, in
`DQ_SerialManipulator`, `fkm(q, n-1)` includes the end effector, whereas
`fkm(q, j)` for `j < n-1` does not. Therefore, the offsets must be computed with
the same `DQ_Kinematics` model (including its end effector) used at runtime.

## 8. `vfi_array`

The entity lists (`entity_*`) contain names defined in `environment_entities` or
`robot_entities`. They are lists to support, in future versions, primitives
described by more than one entity. In V3, each list must contain exactly one
element.

### 8.1 `"ENVIRONMENT_TO_ROBOT"`

| Parameter | Description/Possible values | Type | Required |
|---|---|---|---|
| `vfi_type` | `"ENVIRONMENT_TO_ROBOT"` | string | yes |
| `entity_environment` | `[environment_entity_name]` | string list | yes |
| `entity_robot` | `[robot_entity_name]` | string list | yes |
| `entity_environment_primitive_type` | `"POINT"`, `"LINE"`, `"PLANE"`, `"LINE_ANGLE"` | string | yes |
| `entity_robot_primitive_type` | `"POINT"`, `"LINE_ANGLE"` | string | yes |
| `safe_distance` | Meters. Degrees for `LINE_ANGLE`. | double | yes |
| `buffer` | Meters. Degrees for `LINE_ANGLE`. Default: `0.0`. | double | no |
| `vfi_gain` | Positive gain. | double | yes |
| `direction` | `"RESTRICTED_ZONE"`, `"SAFE_ZONE"` | string | yes |
| `tag` | Unique identifier of the VFI. | string | yes |

Supported combinations (robot primitive – environment primitive):

| Robot | Environment | VFI class |
|---|---|---|
| `POINT` | `POINT` | `RPOINT_TO_POINT` |
| `POINT` | `LINE` | `RPOINT_TO_LINE` |
| `POINT` | `PLANE` | `RPOINT_TO_PLANE` |
| `LINE_ANGLE` | `LINE_ANGLE` | `RLINE_TO_LINE_ANGLE` |

### 8.2 `"ROBOT_TO_ROBOT"`

| Parameter | Description/Possible values | Type | Required |
|---|---|---|---|
| `vfi_type` | `"ROBOT_TO_ROBOT"` | string | yes |
| `entity_one` | `[robot_entity_name]` | string list | yes |
| `entity_two` | `[robot_entity_name]` | string list | yes |
| `entity_one_primitive_type` | `"POINT"` | string | yes |
| `entity_two_primitive_type` | `"POINT"` | string | yes |
| `safe_distance` | Meters. | double | yes |
| `buffer` | Meters. Default: `0.0`. Reserved: the RCM currently ignores it for this type. | double | no |
| `vfi_gain` | Positive gain. | double | yes |
| `direction` | `"RESTRICTED_ZONE"`, `"SAFE_ZONE"`. Reserved: the RCM currently uses `"RESTRICTED_ZONE"`. | string | yes |
| `tag` | Unique identifier of the VFI. | string | yes |

Supported combination: `POINT` – `POINT` (`RPOINT_TO_POINT`).

## 9. Validation rules

A reader must reject the file if any of the following conditions is not met:

1. `vfi_file_version` is `3`.
2. `robots` contains exactly one element, and `dim_configuration` matches the
   `DQ_Kinematics` model.
3. Entity names are unique across `environment_entities` and `robot_entities`.
4. Every name in the `vfi_array` exists in the corresponding entity table:
   `entity_environment` in `environment_entities`; `entity_robot`, `entity_one`,
   and `entity_two` in `robot_entities`.
5. Each entity list in the `vfi_array` contains exactly one element.
6. `joint_index` and `robot_index` are within the ranges of Sections 4 and 7.
7. Each `pose`/`offset` is either a mapping with `translation` (3 numbers) and
   `rotation` (4 numbers), or a sequence of 8 numbers, and is a unit dual
   quaternion within the tolerance of Section 5.1.3.
8. `attached_direction`, primitive types, and `direction` take one of the listed values.
9. The primitive combination is supported (Sections 8.1 and 8.2).
10. Tags are unique.

## 10. Differences with respect to V2

| V2 | V3 |
|---|---|
| `cs_entity_environment`, `cs_entity_robot` | `entity_environment`, `entity_robot` |
| `cs_entity_one`, `cs_entity_two` | `entity_one`, `entity_two` |
| Entities are CoppeliaSim object names | Entities are names defined in `environment_entities`/`robot_entities` |
| Poses/offsets obtained from CoppeliaSim | `pose`/`offset` stored in the file |
| Attached direction hard-coded to `k_` | `attached_direction` per entity (default `"k_"`) |
| `robot_index`, `joint_index` in each VFI | `robot_index`, `joint_index` in each robot entity |
| `robot_index_one/two`, `joint_index_one/two` in each VFI | Taken from `entity_one`/`entity_two` |
| — | `robots`, `metadata` |
| `"LINESEGMENT"` (three objects) | Not included in V3 |

V2 files remain valid and can still be loaded with the CoppeliaSim-based RCM
constructor.

### 10.1 Migration from V2

A V2 file can be converted into V3 using the CoppeliaSim scene it refers to:

1. For each environment object: `pose = cs->get_object_pose(name)`.
2. For each robot object attached to joint `j`:
   `offset = robot->fkm(q, j).conj() * cs->get_object_pose(name)`, where `q` is
   the current robot configuration in the scene.
3. Set `attached_direction: "k_"` (V2 behavior).
4. Move `robot_index`/`joint_index` from the VFIs to the robot entities, and
   rename the `cs_entity_*` keys. If the same object is attached to different
   joints in different VFIs, create one robot entity per joint.
