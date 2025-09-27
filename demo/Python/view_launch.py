import mujoco
import mujoco.viewer


mujoco.mj_loadPluginLibrary('/home/albusgive2/software/mujoco-3.3.6/build/bin/mujoco_plugin/libsensor_ray.so')
m = mujoco.MjModel.from_xml_path('../../ray_caster.xml')
d = mujoco.MjData(m)
mujoco.viewer.launch(m, d)

