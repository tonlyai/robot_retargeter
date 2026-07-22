# Grogu Robot Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `asset/robot/grogu` as a target robot in the robot_retargeter pipeline by extending its MJCF with virtual reference bodies and adding a matching `config/robot/grogu.yaml`.

**Architecture:** Follow the same pattern used by `g1`, `h1`, and `t800`: add invisible collision-free sphere bodies to the MJCF for `hips_sphere`, `neck_sphere`, `head_sphere`, and heel contact probes, then create a YAML config that maps canonical skeleton links to these bodies and the existing grogu links.

**Tech Stack:** MuJoCo, YAML, Python 3.11, project scripts `smpl_replay.py` and `robot_retarget.py`.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `asset/robot/grogu/mjcf/grogu.xml` | Modify | Add virtual reference bodies required by the retargeting config. |
| `config/robot/grogu.yaml` | Create | Robot-specific retargeting configuration. |
| `scripts/verify_grogu_mjcf.py` | Create (optional validation helper) | Quick MuJoCo load + body-name sanity check. |

---

### Task 1: Add virtual reference bodies to `asset/robot/grogu/mjcf/grogu.xml`

**Files:**
- Modify: `asset/robot/grogu/mjcf/grogu.xml`

- [ ] **Step 1: Add `hips_sphere` inside `torso_link`**

Replace this snippet inside `torso_link` (immediately after the `torso_link_visual` geom):

```xml
      <geom name="torso_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="" type="mesh" mesh="torso_link.STL" class="visual" />
      <body name="left_hip_roll_link" pos="-0.039396 0.0136 -0.034598" quat="1.0 0.0 0.0 0.0">
```

with:

```xml
      <geom name="torso_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="" type="mesh" mesh="torso_link.STL" class="visual" />
      <body name="hips_sphere" pos="0 0 -0.0346" quat="1.0 0.0 0.0 0.0">
        <geom name="hips_sphere_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="1 0 0 0" class="visual" />
      </body>
      <body name="left_hip_roll_link" pos="-0.039396 0.0136 -0.034598" quat="1.0 0.0 0.0 0.0">
```

- [ ] **Step 2: Add `left_foot_end_link` inside `left_ankle_pitch_link`**

Replace:

```xml
                <body name="left_toe_link" pos="0.035 0.0 -0.025" quat="1.0 0.0 0.0 0.0">
                  <geom name="left_toe_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="1 0 0 1" class="visual" />
                  <site name="left_toe_link_site" pos="0 0 0" quat="1 0 0 0" />
                </body>
```

with:

```xml
                <body name="left_foot_end_link" pos="-0.035 0.0 -0.025" quat="1.0 0.0 0.0 0.0">
                  <geom name="left_foot_end_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="1 0 0 0" class="visual" />
                </body>
                <body name="left_toe_link" pos="0.035 0.0 -0.025" quat="1.0 0.0 0.0 0.0">
                  <geom name="left_toe_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="1 0 0 1" class="visual" />
                  <site name="left_toe_link_site" pos="0 0 0" quat="1 0 0 0" />
                </body>
```

- [ ] **Step 3: Add `right_foot_end_link` inside `right_ankle_pitch_link`**

Replace:

```xml
                <body name="right_toe_link" pos="0.035 0.0 -0.025" quat="1.0 0.0 0.0 0.0">
                  <geom name="right_toe_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="1 0 0 1" class="visual" />
                  <site name="right_toe_link_site" pos="0 0 0" quat="1 0 0 0" />
                </body>
```

with:

```xml
                <body name="right_foot_end_link" pos="-0.035 0.0 -0.025" quat="1.0 0.0 0.0 0.0">
                  <geom name="right_foot_end_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="1 0 0 0" class="visual" />
                </body>
                <body name="right_toe_link" pos="0.035 0.0 -0.025" quat="1.0 0.0 0.0 0.0">
                  <geom name="right_toe_link_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="1 0 0 1" class="visual" />
                  <site name="right_toe_link_site" pos="0 0 0" quat="1 0 0 0" />
                </body>
```

- [ ] **Step 4: Add `neck_sphere` and `head_sphere` inside `torso_link`**

Replace:

```xml
      </body>
      <site name="torso_link_site" pos="0 0 0" quat="1 0 0 0" />
```

with:

```xml
      </body>
      <body name="neck_sphere" pos="-0.049 0 0.058" quat="1.0 0.0 0.0 0.0">
        <geom name="neck_sphere_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="0.6 0.6 0.6 0" class="visual" />
        <body name="head_sphere" pos="0 0 0.12" quat="1.0 0.0 0.0 0.0">
          <geom name="head_sphere_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" type="sphere" size="0.005" rgba="0.6 0.6 0.6 0" class="visual" />
        </body>
      </body>
      <site name="torso_link_site" pos="0 0 0" quat="1 0 0 0" />
```

- [ ] **Step 5: Verify the MJCF still parses**

Run:

```bash
python -c "import mujoco; mujoco.MjModel.from_xml_path('asset/robot/grogu/mjcf/grogu.xml'); print('MJCF parses OK')"
```

Expected output:

```text
MJCF parses OK
```

- [ ] **Step 6: Commit MJCF changes**

```bash
git add asset/robot/grogu/mjcf/grogu.xml
git commit -m "feat(grogu): add virtual reference bodies for retargeting"
```

---

### Task 2: Create `config/robot/grogu.yaml`

**Files:**
- Create: `config/robot/grogu.yaml`

- [ ] **Step 1: Write the config file**

Create `config/robot/grogu.yaml` with the following content:

```yaml
robot_xml_path: "asset/robot/grogu/mjcf/grogu.xml"
verbose: True
render_debug: False
keypoints_path: "output_data/keypoints/grogu/Form_1_stageii_keypoints.pkl"

joints_limit_offset_degrees:
  knee_joint: [10.0, 0.0]
  elbow_joint: [10.0, 0.0]

knee_angle_offset_degrees: 15.0

robot_links:
  left_hip: [hips_sphere, left_hip_roll_link]
  left_thigh: [left_hip_roll_link, left_knee_link]
  left_calf: [left_knee_link, left_ankle_pitch_link]

  right_hip: [hips_sphere, right_hip_roll_link]
  right_thigh: [right_hip_roll_link, right_knee_link]
  right_calf: [right_knee_link, right_ankle_pitch_link]

  neck: [hips_sphere, neck_sphere]
  head: [neck_sphere, head_sphere]

  left_shoulder: [neck_sphere, left_shoulder_pitch_link]
  left_arm: [left_shoulder_pitch_link, left_elbow_link]
  left_fore_arm: [left_elbow_link, left_wrist_yaw_link]

  right_shoulder: [neck_sphere, right_shoulder_pitch_link]
  right_arm: [right_shoulder_pitch_link, right_elbow_link]
  right_fore_arm: [right_elbow_link, right_wrist_yaw_link]

contact_links: [left_foot_end_link, left_toe_link,
                right_foot_end_link, right_toe_link,
                left_wrist_yaw_link, right_wrist_yaw_link]

contact_vel_calculate_window: 6
contact_vel_threshold: 0.5
contact_height_threshold: 0.05
contact_height_lpf_alpha: 0.15
contact_pos_fixed_factor: 15.0

ik_match_table: {
  "hips_mean": ["hips_sphere", 100, 0],

  "left_hip": ["left_hip_roll_link", 30, 3],
  "left_thigh": ["left_knee_link", 0.0, 3.0],
  "left_calf": ["left_ankle_pitch_link", 30, 3],

  "right_hip": ["right_hip_roll_link", 30, 3],
  "right_thigh": ["right_knee_link", 0.0, 3.0],
  "right_calf": ["right_ankle_pitch_link", 30, 3],

  "head": ["head_sphere", 0.0, 3.0],

  "left_shoulder": ["left_shoulder_pitch_link", 30, 3],
  "left_arm": ["left_elbow_link", 10.0, 1],
  "left_fore_arm": ["left_wrist_yaw_link", 10, 1],

  "right_shoulder": ["right_shoulder_pitch_link", 30, 3],
  "right_arm": ["right_elbow_link", 10.0, 1],
  "right_fore_arm": ["right_wrist_yaw_link", 10, 1]
}

key_frame_config:
  hips_mean:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  left_up_leg:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  left_leg:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  left_foot:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  right_up_leg:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  right_leg:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  right_foot:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  shoulder_mean:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
  left_arm:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [0.0, 1.0, 0.0]
      z: [-1.0, 0.0, 0.0]
  left_fore_arm:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [1.0, 0.0, 0.0]
      y: [0.0, 1.0, 0.0]
      z: [0.0, 0.0, 1.0]
  left_hand:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [1.0, 0.0, 0.0]
      y: [0.0, 1.0, 0.0]
      z: [0.0, 0.0, 1.0]
  right_arm:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [0.0, -1.0, 0.0]
      z: [1.0, 0.0, 0.0]
  right_fore_arm:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [-1.0, 0.0, 0.0]
      y: [0.0, -1.0, 0.0]
      z: [0.0, 0.0, 1.0]
  right_hand:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [-1.0, 0.0, 0.0]
      y: [0.0, -1.0, 0.0]
      z: [0.0, 0.0, 1.0]
  head:
    offset_deg_xyz: [0.0, 0.0, 0.0]
    axis_map_cols:
      x: [0.0, 0.0, 1.0]
      y: [1.0, 0.0, 0.0]
      z: [0.0, 1.0, 0.0]
```

- [ ] **Step 2: Sanity-check the YAML**

Run:

```bash
python -c "import yaml; yaml.safe_load(open('config/robot/grogu.yaml')); print('YAML parses OK')"
```

Expected output:

```text
YAML parses OK
```

- [ ] **Step 3: Commit the config**

```bash
git add config/robot/grogu.yaml
git commit -m "feat(grogu): add retargeting config"
```

---

### Task 3: Verify all referenced bodies exist in the MJCF

**Files:**
- Create: `scripts/verify_grogu_mjcf.py`

- [ ] **Step 1: Write the verification script**

Create `scripts/verify_grogu_mjcf.py`:

```python
import mujoco
import yaml
from pathlib import Path

REQUIRED_BODIES = {
    "hips_sphere",
    "neck_sphere",
    "head_sphere",
    "left_hip_roll_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_foot_end_link",
    "left_toe_link",
    "left_shoulder_pitch_link",
    "left_elbow_link",
    "left_wrist_yaw_link",
    "right_hip_roll_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_foot_end_link",
    "right_toe_link",
    "right_shoulder_pitch_link",
    "right_elbow_link",
    "right_wrist_yaw_link",
}

CONFIG_PATH = Path("config/robot/grogu.yaml")
ROBOT_XML_PATH = Path("asset/robot/grogu/mjcf/grogu.xml")


def main():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    xml_path = Path(config["robot_xml_path"])
    if not xml_path.is_absolute():
        xml_path = CONFIG_PATH.parent.parent / xml_path

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    missing = [name for name in REQUIRED_BODIES if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0]

    if missing:
        print("Missing bodies:", missing)
        raise SystemExit(1)

    print(f"All {len(REQUIRED_BODIES)} required bodies found in MJCF")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the verification**

Run:

```bash
python scripts/verify_grogu_mjcf.py
```

Expected output:

```text
All 19 required bodies found in MJCF
```

- [ ] **Step 3: Commit the verification script**

```bash
git add scripts/verify_grogu_mjcf.py
git commit -m "test(grogu): add MJCF body sanity check"
```

---

### Task 4: End-to-end smoke test with SMPL-X motion

**Files:**
- Uses: `scripts/smpl_replay.py`, `scripts/robot_retarget.py`

- [ ] **Step 1: Generate keypoints for grogu from an SMPL-X motion**

Run:

```bash
python scripts/smpl_replay.py \
  --motion_file dataset/ACCAD/Form_1_stageii.npz \
  --robot-config config/robot/grogu.yaml \
  --fps 30 \
  --no-viewer
```

Expected result: The script exits with code 0 and creates `output_data/keypoints/grogu/Form_1_stageii_keypoints.pkl`.

- [ ] **Step 2: Retarget the keypoints onto grogu**

Run:

```bash
python scripts/robot_retarget.py \
  --config config/robot/grogu.yaml \
  --keypoints-name Form_1_stageii \
  --no-render-debug
```

Expected result: The script exits with code 0 and creates `output_data/robot_motion/Form_1_stageii_grogu.csv`.

- [ ] **Step 3: Verify output files exist**

Run:

```bash
ls -lh output_data/keypoints/grogu/Form_1_stageii_keypoints.pkl
ls -lh output_data/robot_motion/Form_1_stageii_grogu.csv
```

Expected output: Both files exist with non-zero size.

- [ ] **Step 4: Commit a note about validation (optional)**

No code change is required for validation. If the smoke test passed, no additional commit is needed.

---

## Self-Review

- **Spec coverage:**
  - Virtual `hips_sphere`, `neck_sphere`, `head_sphere` bodies → Task 1, Steps 1 and 4.
  - Virtual `left_foot_end_link` and `right_foot_end_link` → Task 1, Steps 2 and 3.
  - `config/robot/grogu.yaml` matching g1/h1 conventions → Task 2.
  - Validation by loading config/MJCF and running retargeting → Tasks 3 and 4.
- **Placeholder scan:** No TBD/TODO/fill-in-details; every step contains exact XML, YAML, or command.
- **Type consistency:** All body names in the YAML match the bodies added to the MJCF and the verification script.
