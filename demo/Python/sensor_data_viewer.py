import mujoco
import mujoco_viewer    #pip install mujoco-python-viewer
import cv2
import numpy as np

mujoco.mj_loadPluginLibrary('../../lib/libsensor_ray.so')

m = mujoco.MjModel.from_xml_path(
    "../../model/ray_caster.xml"
)

d = mujoco.MjData(m)

def get_ray_caster_info(model: mujoco.MjModel, data: mujoco.MjData, sensor_name: str):
    data_ps = []
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id == -1:
        print("Sensor not found")
        return 0, 0, data_ps
    sensor_plugin_id = model.sensor_plugin[sensor_id]
    state_idx = model.plugin_stateadr[sensor_plugin_id]
    state_num = model.plugin_statenum[sensor_plugin_id]
    for i in range(state_idx + 2, state_idx + state_num, 2):
        if i + 1 < len(data.plugin_state):
            data_ps.append((int(data.plugin_state[i]), int(data.plugin_state[i + 1])))
    h_ray_num = (
        int(data.plugin_state[state_idx]) if state_idx < len(data.plugin_state) else 0
    )
    v_ray_num = (
        int(data.plugin_state[state_idx + 1])
        if state_idx + 1 < len(data.plugin_state)
        else 0
    )
    return h_ray_num, v_ray_num, data_ps


def get_sensor_data(sensor_name):
    sensor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id == -1:
        raise ValueError(f"Sensor '{sensor_name}' not found in model!")
    start_idx = m.sensor_adr[sensor_id]
    dim = m.sensor_dim[sensor_id]
    sensor_values = d.sensordata[start_idx : start_idx + dim]
    return sensor_values


h_rays, v_rays, pairs = get_ray_caster_info(m, d, "raycastercamera")
print(f"h_rays: {h_rays}")
print(f"v_rays: {v_rays}")
print(f"[data_point,data_size]: {pairs}")
image_id = 1

# 创建渲染器
viewer = mujoco_viewer.MujocoViewer(m, d)
# 模拟循环
while viewer.is_alive:
    mujoco.mj_step(m, d)

    img = d.sensor("raycastercamera").data
    image = np.array(img[pairs[0][0]:pairs[0][0]+pairs[0][1]], dtype=np.uint8).reshape(v_rays, h_rays)
    inv_image = np.array(img[pairs[1][0]:pairs[1][0]+pairs[1][1]], dtype=np.uint8).reshape(v_rays, h_rays)
    cv2.resize(image, (h_rays*10, v_rays*10), interpolation=cv2.INTER_NEAREST)
    cv2.resize(inv_image, (h_rays*10, v_rays*10), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("image", image)
    cv2.imshow("inv_image", inv_image)
    cv2.waitKey(1)
    viewer.render()
