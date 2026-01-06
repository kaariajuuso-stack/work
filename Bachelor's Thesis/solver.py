import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("solver.xml")
model.opt.iterations = 10
data = mujoco.MjData(model)

while data.time < 180:
    mujoco.mj_step(model, data)
        
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        viewer.sync()