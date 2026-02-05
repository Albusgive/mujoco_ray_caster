import mujoco
import mujoco_viewer
import cv2
import numpy as np

mujoco.mj_loadPluginLibrary('../../lib/libsensor_raycaster.so')

m = mujoco.MjModel.from_xml_path("../../model/ray_caster3.xml")
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
    h_ray_num = int(data.plugin_state[state_idx]) if state_idx < len(data.plugin_state) else 0
    v_ray_num = int(data.plugin_state[state_idx + 1]) if state_idx + 1 < len(data.plugin_state) else 0
    return h_ray_num, v_ray_num, data_ps

sensor_name = "raycastercamera"
h_rays, v_rays, pairs = get_ray_caster_info(m, d, sensor_name)
print(f"h_rays: {h_rays}")
print(f"v_rays: {v_rays}")
print(f"[data_point,data_size]: {pairs}")

viewer = mujoco_viewer.MujocoViewer(m, d,
            width=1920,
            height=1080,)

def visualize_block(block_data, length, idx, h_rays, v_rays, win_prefix="block"):
    """
    block_data: 1D numpy array
    length:     数据长度
    idx:        第几个 block
    """
    # 尝试用 (v_rays, h_rays[, C]) 的方式 reshape
    hw = h_rays * v_rays
    img = None

    if length == hw:
        # 单通道图像
        img = block_data.reshape(v_rays, h_rays)
    elif length == hw * 3:
        # 3 通道图（RGB 或其他）
        img = block_data.reshape(v_rays, h_rays, 3)
    elif length % hw == 0 and length // hw <= 4:
        # 多通道图像 (最多 4 通道，超过就不凑了)
        c = length // hw
        img = block_data.reshape(v_rays, h_rays, c)
        # 如果不是 3 通道，就只显示第 1 通道，避免 cv2 报错
        if c != 3:
            img = img[:, :, 0]
    else:
        # 实在对不上 (h_rays, v_rays)，就铺成近似正方形的热力图
        side = int(np.ceil(np.sqrt(length)))
        tmp = np.zeros((side * side,), dtype=np.float32)
        tmp[:length] = block_data
        img = tmp.reshape(side, side)

    # 归一化到 [0,255] 便于显示
    img = img.astype(np.float32)
    vmin, vmax = img.min(), img.max()
    if vmax > vmin:
        img_norm = (img - vmin) / (vmax - vmin)
    else:
        img_norm = np.zeros_like(img)
    img_uint8 = (img_norm * 255).astype(np.uint8)

    # 放大一点看得清楚
    scale = 4
    h, w = img_uint8.shape[:2]
    img_show = cv2.resize(
        img_uint8, (w * scale, h * scale),
        interpolation=cv2.INTER_NEAREST
    )

    win_name = f"{win_prefix}_{idx}_len{length}"
    cv2.imshow(win_name, img_show)


while viewer.is_alive:
    mujoco.mj_step(m, d)
    viewer.render()

    # 传感器完整数据（一维 float 数组）
    full_data = d.sensor(sensor_name).data

    # 遍历 get_ray_caster_info 给出的所有数据段
    for i, (start, size) in enumerate(pairs):
        block = np.array(full_data[start:start+size], dtype=np.float32)
        visualize_block(block, size, i, h_rays, v_rays, win_prefix=sensor_name)

    # ESC 退出
    if cv2.waitKey(1) & 0xFF == 27:
        break

cv2.destroyAllWindows()
viewer.close()
