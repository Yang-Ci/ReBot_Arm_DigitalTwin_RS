import json
from types import SimpleNamespace

import mujoco

from rebotarm_mujoco_rs.mujoco_sync import RsMujocoSync, find_default_model


def test_object_state_reports_physical_table_contact_and_speed():
    model = mujoco.MjModel.from_xml_path(str(find_default_model()))
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)

    name = 'yellow_cylinder'
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    messages = []
    node = SimpleNamespace(
        model=model, data=data, object_body_ids={name: body_id},
        object_publisher=SimpleNamespace(publish=messages.append),
        last_object_publish_time=0.0, object_publish_period=0.0,
        simulation_mode='physics',
    )
    RsMujocoSync._publish_object_states(node)
    resting = json.loads(messages[-1].data)['objects'][0]
    assert resting['table_contact'] is True
    assert resting['linear_speed_mps'] < 0.025

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'yellow_cylinder_freejoint')
    data.qpos[model.jnt_qposadr[joint_id] + 2] += 0.1
    mujoco.mj_forward(model, data)
    RsMujocoSync._publish_object_states(node)
    lifted = json.loads(messages[-1].data)['objects'][0]
    assert lifted['table_contact'] is False
