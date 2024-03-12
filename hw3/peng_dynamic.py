import xml.etree.ElementTree as ET 
import mujoco as mj
import mujoco_viewer
import numpy as np
import random
import math

# create elements/subelements
mujoco = ET.Element('mujoco')

# option = ET.SubElement(mujoco, 'option')
# option.text = 'timestep="0.1"'

worldbody = ET.SubElement(mujoco, 'worldbody')
actuator = ET.SubElement(mujoco, 'actuator')

# create plane and light
light = ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
plane = ET.SubElement(worldbody, 'geom', name="ground", type="plane", size="5 5 0.1", rgba=".9 .9 .9 1")

# create penguin body (this is constant and always exists as such)
bf = ET.SubElement(worldbody, 'body', name="body")
joint_free = ET.SubElement(bf, 'freejoint', name="root")
pen_body = ET.SubElement(bf, 'geom', name="torso", type="box", size="0.5 0.25 0.25", pos="0 0 0.25", rgba="0 0 1 1", mass="0.01")

# create an empty array for all wing parts to actuate later
wing_list = []

# randomize number of wings
num_wings = random.randint(2, 5)
print(f"Number of wings: {num_wings}")
# create and attach hinge joints for each wing
for i in range(num_wings):
    # attach wing0 to body
    if i==0:
        wing = ET.SubElement(bf, 'body', name="wing" + str(i), pos="0 0.4 0.25", euler="0 0 90")
        joint_hinge = ET.SubElement(wing, 'joint', name="wing" + str(i) + "link", type="hinge", axis="0 1 0", range="-15 15")
        wing_geom = ET.SubElement(wing, 'geom', name="wing" + str(i), type="cylinder", size="0.2 0.0325", pos="0 0 0", rgba="0 1 0 1", mass="1")
        # append wing to wing_list
        wing_list.append(wing)
        # print(f"wing_list: {wing_list}")
        continue
    # attach each wing to the previous wing
    
    if i>0:
        
        wing = ET.SubElement(wing_list[i-1], 'body', name="wing" + str(i), euler="0 0 0")
        joint_hinge = ET.SubElement(wing_list[i-1], 'joint', name="wing" + str(i) + "link", type="hinge", pos="0 0 0", axis="0 1 0", range="-15 15")
        wing_geom = ET.SubElement(wing_list[i-1], 'geom', name="wing" + str(i), type="cylinder", size="0.2 0.0325", pos="0 0.4 0", rgba="0 1 0 1", mass="1.5")
        wing_list.append(wing)
        print(f"wing_list: {wing_list}")
        

# create actuator for each wing
for i in range(num_wings):
    motor = ET.SubElement(actuator, 'motor', name="wing" + str(i) + "m", joint=f"wing{i}link", gear="8", ctrllimited="true", ctrlrange="-5 5")
# write to xml file
tree = ET.ElementTree(mujoco)
tree.write('penguin.xml')

# create model and data
model = mj.MjModel.from_xml_path('penguin.xml')
data = mj.MjData(model)

# create viewer
viewer = mujoco_viewer.MujocoViewer(model, data)

# set frequency and amplitude
f = 0.1
amp = 1.5

# Simulation loop
for k in range(10000):
    if viewer.is_alive:
        mj.mj_step(model, data)
        sin_wave = amp * math.sin(2 * math.pi * f * k)
        for i in range(num_wings):
            data.ctrl[i] = sin_wave
        viewer.render()
    else:
        break

# display the number of wings
print(f"Number of wings: {num_wings}")
# display distance of organism
# print(f"Distance of organism: {data.qpos[0]}")
# fitness = data.qpos[0]/num_wings
# # display fitness of organism
# print(f"Fitness of organism: {fitness}")


# Close the viewer
viewer.close()