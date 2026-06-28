from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


ARM_JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
ARM_ACTUATOR_NAMES = tuple(f"{name}_motor" for name in ARM_JOINT_NAMES)
FINGER_R_JOINT_NAME = "finger_r"
FINGER_L_JOINT_NAME = "finger_l"
GRIPPER_ACTUATOR_NAME = "finger_r_motor"
TASK_OBJECT_BODY = "block_red"
TASK_OBJECT_JOINT = "block_red_freejoint"
BASKET_BODY = "basket"
TARGET_GEOM = "basket_target_marker"
TARGET_SITE = "basket_target_site"
EE_SITE = "ee_site"
GRIPPER_CENTER_SITE = "gripper_center_site"


@dataclass
class NameMap:
    arm_joint_names: tuple[str, ...]
    arm_actuator_names: tuple[str, ...]
    controlled_joint_names: tuple[str, ...]
    arm_qposadr: np.ndarray
    arm_dofadr: np.ndarray
    controlled_qposadr: np.ndarray
    controlled_dofadr: np.ndarray
    arm_actuator_ids: np.ndarray
    gripper_actuator_id: int
    finger_r_joint_id: int
    finger_l_joint_id: int
    finger_r_qposadr: int
    finger_l_qposadr: int
    finger_r_dofadr: int
    finger_l_dofadr: int
    object_body_id: int
    object_joint_id: int
    object_qposadr: int
    object_dofadr: int
    basket_body_id: int
    target_body_id: int
    target_geom_id: int
    target_site_id: int
    ee_site_id: int
    gripper_center_site_id: int
    joint_ranges: np.ndarray
    actuator_ctrlrange: np.ndarray


def resolve_name_map(model: mujoco.MjModel) -> NameMap:
    arm_joint_ids = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINT_NAMES]
    finger_r_joint_id = _id(model, mujoco.mjtObj.mjOBJ_JOINT, FINGER_R_JOINT_NAME)
    finger_l_joint_id = _id(model, mujoco.mjtObj.mjOBJ_JOINT, FINGER_L_JOINT_NAME)
    controlled_joint_ids = arm_joint_ids + [finger_r_joint_id]

    arm_actuator_ids = np.array(
        [_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ARM_ACTUATOR_NAMES],
        dtype=np.int32,
    )
    gripper_actuator_id = _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, GRIPPER_ACTUATOR_NAME)

    object_joint_id = _id(model, mujoco.mjtObj.mjOBJ_JOINT, TASK_OBJECT_JOINT)
    object_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, TASK_OBJECT_BODY)
    basket_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, BASKET_BODY)
    target_geom_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, TARGET_GEOM)
    target_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, TARGET_SITE)
    ee_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
    gripper_center_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, GRIPPER_CENTER_SITE)

    controlled_qposadr = np.array([model.jnt_qposadr[jid] for jid in controlled_joint_ids], dtype=np.int32)
    controlled_dofadr = np.array([model.jnt_dofadr[jid] for jid in controlled_joint_ids], dtype=np.int32)
    arm_qposadr = controlled_qposadr[: len(ARM_JOINT_NAMES)]
    arm_dofadr = controlled_dofadr[: len(ARM_JOINT_NAMES)]
    joint_ranges = np.array([model.jnt_range[jid] for jid in controlled_joint_ids], dtype=np.float64)

    actuator_ids = np.concatenate([arm_actuator_ids, np.array([gripper_actuator_id], dtype=np.int32)])
    actuator_ctrlrange = np.array([model.actuator_ctrlrange[aid] for aid in actuator_ids], dtype=np.float64)

    return NameMap(
        arm_joint_names=ARM_JOINT_NAMES,
        arm_actuator_names=ARM_ACTUATOR_NAMES,
        controlled_joint_names=ARM_JOINT_NAMES + (FINGER_R_JOINT_NAME,),
        arm_qposadr=arm_qposadr,
        arm_dofadr=arm_dofadr,
        controlled_qposadr=controlled_qposadr,
        controlled_dofadr=controlled_dofadr,
        arm_actuator_ids=arm_actuator_ids,
        gripper_actuator_id=gripper_actuator_id,
        finger_r_joint_id=finger_r_joint_id,
        finger_l_joint_id=finger_l_joint_id,
        finger_r_qposadr=int(model.jnt_qposadr[finger_r_joint_id]),
        finger_l_qposadr=int(model.jnt_qposadr[finger_l_joint_id]),
        finger_r_dofadr=int(model.jnt_dofadr[finger_r_joint_id]),
        finger_l_dofadr=int(model.jnt_dofadr[finger_l_joint_id]),
        object_body_id=object_body_id,
        object_joint_id=object_joint_id,
        object_qposadr=int(model.jnt_qposadr[object_joint_id]),
        object_dofadr=int(model.jnt_dofadr[object_joint_id]),
        basket_body_id=basket_body_id,
        target_body_id=basket_body_id,
        target_geom_id=target_geom_id,
        target_site_id=target_site_id,
        ee_site_id=ee_site_id,
        gripper_center_site_id=gripper_center_site_id,
        joint_ranges=joint_ranges,
        actuator_ctrlrange=actuator_ctrlrange,
    )


def _id(model: mujoco.MjModel, objtype: mujoco.mjtObj, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise KeyError(f"MuJoCo object not found: {objtype} {name}")
    return int(obj_id)
