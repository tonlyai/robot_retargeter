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

CONFIG_PATH = Path(__file__).parent.parent / "config/robot/grogu.yaml"


def main():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    xml_path = Path(config["robot_xml_path"])
    if not xml_path.is_absolute():
        xml_path = CONFIG_PATH.parent.parent.parent / xml_path

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    missing = [
        name
        for name in REQUIRED_BODIES
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0
    ]

    if missing:
        print("Missing bodies:", missing)
        raise SystemExit(1)

    print(f"All {len(REQUIRED_BODIES)} required bodies found in MJCF")


if __name__ == "__main__":
    main()
