import mujoco
import mujoco_viewer
import numpy as np
import dm_control.mujoco
import math

model = dm_control.mujoco.MjModel.from_xml_path('penguin.xml')
data = dm_control.mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)

f = 0.1
amp = 1.5

# Simulation loop
for i in range(10000):
    if viewer.is_alive:
        dm_control.mujoco.mj_step(model, data)
        sin_wave = amp * math.sin(2 * math.pi * f * i)
        data.ctrl[:] = np.array([sin_wave, -sin_wave*2])
        viewer.render()
    else:
        break

# Close the viewer
viewer.close()