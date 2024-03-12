import xml.etree.ElementTree as ET 
import mujoco as mj
import mujoco_viewer
import numpy as np
import random
import math
import matplotlib.pyplot as plt

run_num_list =[]
overall_fitness_list = []

for run in range(100):
    print(f"Run: {run+1}")
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
    pen_body = ET.SubElement(bf, 'geom', name="torso", type="box", size="0.5 0.25 0.25", pos="0 0 0.25", rgba="0 0 1 1", mass="0.1")

    # create an empty array for all wing parts to actuate later
    wing_list = []
    range_list = []

    # randomize number of wings
    num_wings = random.randint(1, 9)
    print(f"Number of wings: {num_wings}")
    # create and attach hinge joints for each wing
    for i in range(num_wings):
        # attach wing0 to body
        if i==0:
            wing = ET.SubElement(bf, 'body', name="wing" + str(i), pos="0 0.4 0.25", euler="0 0 90")
            joint_hinge = ET.SubElement(wing, 'joint', name="wing" + str(i) + "link", type="hinge", axis="0 0 1", range="-1 1")
            wing_geom = ET.SubElement(wing, 'geom', name="wing" + str(i), type="cylinder", size="0.2 0.0325", pos="0 0 0", rgba="0 1 0 1", mass="1")
            # append wing to wing_list
            wing_list.append(wing)
            # print(f"wing_list: {wing_list}")
            continue 
        
        # attach each wing to the previous wing
        if i>0:
            #randomize range of wing movement
            ran_range = random.randint(1, 100)
            range_list.append(ran_range)
            wing = ET.SubElement(wing_list[i-1], 'body', name="wing" + str(i), euler="0 0 0")
            joint_hinge = ET.SubElement(wing_list[i-1], 'joint', name="wing" + str(i) + "link", type="hinge", pos="0 0 0", axis="1 0 0", range=f"{-ran_range} {ran_range}")
            wing_geom = ET.SubElement(wing_list[i-1], 'geom', name="wing" + str(i), type="cylinder", size="0.2 0.0325", pos=f"{0.4*i} 0 0", rgba="0 1 0 1", mass="1.5")
            wing_list.append(wing)
            # print(f"wing_list: {wing_list}")
            
    gear_list = []
    # create actuator for each wing
    for i in range(num_wings):
        # randomize gear for each wing
        ran_gear = random.randint(1, 100)
        gear_list.append(ran_gear)
        motor = ET.SubElement(actuator, 'motor', name="wing" + str(i) + "m", joint=f"wing{i}link", gear=f"{ran_gear}", ctrllimited="true", ctrlrange="-5 5")
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
    for k in range(100):
        if viewer.is_alive:
            mj.mj_step(model, data)
            sin_wave = amp * math.sin(2 * math.pi * f * k)
            for i in range(num_wings):
                data.ctrl[i] = sin_wave
            viewer.render()
        else:
            break

    # distance traveled by organism
    print(f"Distance travelled: {abs(data.qpos[0])}")
    dist_travelled = abs(data.qpos[0])

    # height of organism
    print(f"Height gained: {data.qpos[2]}")
    height_gained = abs(data.qpos[2])

    # calculate average gear power from gear list
    avg_gear = sum(gear_list)/len(gear_list)
    print(f"Average gear power: {avg_gear}")

    # calculate average range of wing movement from range list
    if len(range_list) == 0:
        avg_range = sum(range_list)
    else:
        avg_range = sum(range_list)/len(range_list)
    print(f"Average range of wing movement: {avg_range}")
    # calculate fitness of organism based on distance, number of wings, and gear power
    dist_fitness = (abs(data.qpos[0])/num_wings) * (avg_gear + avg_range)
    height_fitness = (abs(data.qpos[2])/num_wings) * (avg_gear + avg_range)
    fitness = dist_fitness + height_fitness
    print(f"Overall fitness of organism: {fitness}")

    
    run_num_list.append(run+1)
    overall_fitness_list.append(fitness)

    print("-----------------------------------------------------")


    # Close the viewer
    viewer.close()

# plot fitness of each run
plt.scatter(run_num_list, overall_fitness_list, color='blue', marker='o', label='Fitness Points')
plt.plot(run_num_list, overall_fitness_list, color='green', linestyle='-', marker='', label='Fitness Line')
for i, txt in enumerate(overall_fitness_list):
    plt.text(run_num_list[i] + 0.1, overall_fitness_list[i], f'{txt:.2f}', fontsize=8, color='black')
plt.xlabel('Organism Number')
plt.ylabel('Overall Fitness')
plt.title('Fitness of Dynamically Generated Organisms')
plt.show()