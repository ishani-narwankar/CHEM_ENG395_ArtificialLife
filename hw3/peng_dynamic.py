import xml.etree.ElementTree as ET 
import mujoco as mj
import mujoco_viewer
import numpy as np
import random
import math
import matplotlib.pyplot as plt

run_num_list =[]
overall_fitness_list = []
dist_travelled_list = []
height_gained_list = []
avg_gear_list = []
avg_range_list = []
run_params = []

# Initialize variables to store parameters of the most fit organism
most_fit_params = None
most_fit_fitness = -1

for run in range(3):
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
    viewer.cam.distance = 15
    viewer.cam.azimuth = 60

    # set frequency and amplitude
    f = 0.1
    amp = 1.5

    # Simulation loop
    for k in range(1000):
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

    # append distance travelled, height gained, average gear power, and average range of wing movement to respective lists
    dist_travelled_list.append(dist_travelled)
    height_gained_list.append(height_gained)
    avg_gear_list.append(avg_gear)
    avg_range_list.append(avg_range)

    print("-----------------------------------------------------")

    # Check if the current organism is the most fit
    if fitness > most_fit_fitness:
        most_fit_fitness = fitness
        most_fit_params = {
            'num_wings': num_wings,
            'range_list': range_list.copy(),  # Make a copy to avoid overwriting
            'gear_list': gear_list.copy()     # the lists in subsequent iterations
        }
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
# plt.show()

# determine index of most fit organism
most_fit_index = overall_fitness_list.index(max(overall_fitness_list))

# Calculate averages for the remaining organisms
dist_travelled_avg = sum(dist_travelled_list[:most_fit_index] + dist_travelled_list[most_fit_index + 1:]) / (len(run_num_list) - 1)
height_gained_avg = sum(height_gained_list[:most_fit_index] + height_gained_list[most_fit_index + 1:]) / (len(run_num_list) - 1)
avg_gear_avg = sum(avg_gear_list[:most_fit_index] + avg_gear_list[most_fit_index + 1:]) / (len(run_num_list) - 1)
avg_range_avg = sum(avg_range_list[:most_fit_index] + avg_range_list[most_fit_index + 1:]) / (len(run_num_list) - 1)

# Get values for the most fit organism
most_fit_dist_travelled = dist_travelled_list[most_fit_index]
most_fit_height_gained = height_gained_list[most_fit_index]
most_fit_avg_gear = avg_gear_list[most_fit_index]
most_fit_avg_range = avg_range_list[most_fit_index]

# plot bar chart comparing average values to the most fit organism
parameters = ['Distance Traveled', 'Height Gained', 'Average Gear Power', 'Average Range of Wing Movement']
avg_values = [dist_travelled_avg, height_gained_avg, avg_gear_avg, avg_range_avg]
most_fit_values = [most_fit_dist_travelled, most_fit_height_gained, most_fit_avg_gear, most_fit_avg_range]

bar_width = 0.35
index = range(len(parameters))

fig, ax = plt.subplots()
bar1 = ax.bar(index, avg_values, bar_width, label='Average Values')
bar2 = ax.bar([i + bar_width for i in index], most_fit_values, bar_width, label='Most Fit Organism', color='orange')

ax.set_xlabel('Parameters')
ax.set_ylabel('Values')
ax.set_title('Comparison of Average Values to Most Fit Organism')
ax.set_xticks([i + bar_width / 2 for i in index])
ax.set_xticklabels(parameters)
ax.legend()

plt.show()

#---------------------------#
# Write most fit organism to XML file

# Write most fit organism to XML file
if most_fit_params is not None:
    # Define the number of wings for the most fit organism
    num_wings = len(most_fit_params['range_list'])

    # Create XML elements for the most fit organism
    most_fit_mujoco = ET.Element('mujoco')
    most_fit_worldbody = ET.SubElement(most_fit_mujoco, 'worldbody')
    most_fit_actuator = ET.SubElement(most_fit_mujoco, 'actuator')

    # Create plane and light
    light = ET.SubElement(most_fit_worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
    plane = ET.SubElement(most_fit_worldbody, 'geom', name="ground", type="plane", size="5 5 0.1", rgba=".9 .9 .9 1")
    bf_mf = ET.SubElement(most_fit_worldbody, 'body', name="body")
    joint_free_mf = ET.SubElement(bf_mf, 'freejoint', name="root")
    pen_body_mf = ET.SubElement(bf_mf, 'geom', name="torso", type="box", size="0.5 0.25 0.25", pos="0 0 0.25", rgba="0 0 1 1", mass="0.1")

    # Add wings to body
    for w in range(num_wings):
        wing_mf = ET.SubElement(bf_mf, 'body', name="wing" + str(w), pos="0 0.4 0.25", euler="0 0 90")
        joint_hinge_mf = ET.SubElement(wing_mf, 'joint', name="wing" + str(w) + "link", type="hinge", axis="0 0 1", range=f"{-most_fit_params['range_list'][w]} {most_fit_params['range_list'][w]}")
        wing_geom_mf = ET.SubElement(wing_mf, 'geom', name="wing" + str(w), type="cylinder", size="0.2 0.0325", pos="0 0 0", rgba="0 1 0 1", mass="1")
        motor_mf = ET.SubElement(most_fit_actuator, 'motor', name="wing" + str(w) + "m", joint=f"wing{w}link", gear=f"{most_fit_params['gear_list'][w]}", ctrllimited="true", ctrlrange="-5 5")

    # Write to XML file
    tree2 = ET.ElementTree(most_fit_mujoco)
    tree2.write('most_fit_penguin.xml')
    print("Most fit organism config written to most_fit_penguin.xml")
else:
    print("No most fit organism found.")