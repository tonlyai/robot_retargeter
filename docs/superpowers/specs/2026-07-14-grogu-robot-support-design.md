# Grogu Robot Support Design

## Goal

Add the `asset/robot/grogu` model to the robot_retargeter pipeline as a **target robot**, so motions from SMPL-X or other source robots can be retargeted onto it.

## Decisions from Brainstorming

- Grogu will be configured as a **target robot only**.
- A **virtual head sphere** will be added because the grogu MJCF has no head link, and we want to keep head orientation tracking.
- **Virtual foot_end links** will be added in addition to the existing toe links, matching the g1/h1 convention for contact detection and foot-sliding suppression.
- The MJCF will be edited **directly** ( Approach A ) rather than creating a separate retargeting MJCF or auto-injecting bodies in scripts.

## Changes

### 1. `asset/robot/grogu/mjcf/grogu.xml`

Add the following virtual marker bodies. They are invisible collision-free spheres used only as IK reference frames, following the pattern already used by `g1`, `h1`, and `t800`.

| Body | Parent | Relative position | Purpose |
|---|---|---|---|
| `hips_sphere` | `torso_link` | `0 0 -0.0346` | Centered between the left/right hip roll joints, target for `hips_mean`. |
| `neck_sphere` | `torso_link` | `-0.049 0 0.058` | Centered between the shoulder pitch joints, anchor for shoulder and head links. |
| `head_sphere` | `neck_sphere` | `0 0 0.12` | Virtual head position for head orientation tracking. |
| `left_foot_end_link` | `left_ankle_pitch_link` | `-0.035 0 -0.025` | Heel contact probe, mirrors the existing `left_toe_link`. |
| `right_foot_end_link` | `right_ankle_pitch_link` | `-0.035 0 -0.025` | Heel contact probe, mirrors the existing `right_toe_link`. |

The positions are derived from the existing joint/body offsets in `grogu.xml` so the virtual bodies align with the robot's actual kinematic topology.

### 2. `config/robot/grogu.yaml` (new file)

A new robot config file modeled on `config/robot/g1.yaml` / `config/robot/h1.yaml`:

- `robot_xml_path`: `asset/robot/grogu/mjcf/grogu.xml`
- `keypoints_path`: `output_data/keypoints/grogu/Form_1_stageii_keypoints.pkl` (default, overridable via `--keypoints-name`)
- `joints_limit_offset_degrees`:
  - `knee_joint: [10.0, 0.0]`
  - `elbow_joint: [10.0, 0.0]`
- `knee_angle_offset_degrees`: `15.0`
- `robot_links`: map the canonical link names to the grogu body names.
- `contact_links`: `left_foot_end_link`, `left_toe_link`, `right_foot_end_link`, `right_toe_link`, `left_wrist_yaw_link`, `right_wrist_yaw_link`.
- `ik_match_table`: high root position weight on `hips_sphere`, per-limb position/orientation weights matching g1/h1, and head orientation-only target on `head_sphere`.
- `key_frame_config`: axis maps and Euler offsets per canonical body, matching the g1/h1 convention.

### 3. Validation

After the files are created, run a headless smoke test with `scripts/robot_retarget.py` using a pre-generated keypoints file. The test must load the config and MJCF without raising `Missing robot body in MJCF` or other configuration errors.

## Out of Scope

- No changes to the retargeting scripts.
- No support for using grogu as a **source** robot.
- No tuning of retargeting quality beyond the default g1/h1-style parameters; quality tuning can be done later if visual inspection shows issues.
