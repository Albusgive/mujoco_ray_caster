import mujoco
import mujoco.viewer


mujoco.mj_loadPluginLibrary('../../lib/libsensor_ray.so')
m = mujoco.MjModel.from_xml_path('../../model/ray_caster.xml')
d = mujoco.MjData(m)
mujoco.viewer.launch(m, d)

