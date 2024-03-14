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

# Initialize variables to store parameters of the most fit organism
most_fit_params = None
most_fit_fitness = -1

for run in range(50):
    print(f"Run: {run+1}")

    mujoco = ET.Element('mujoco')
    worldbody = ET.SubElement(mujoco, 'worldbody')
    actuator = ET.SubElement(mujoco, 'actuator')
    light = ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
    plane = ET.SubElement(worldbody, 'geom', name="ground", type="plane", size="5 5 0.1", rgba=".9 .9 .9 1")
    bf = ET.SubElement(worldbody, 'body', name="body")
    joint_free = ET.SubElement(bf, 'freejoint', name="root")
    pen_body = ET.SubElement(bf, 'geom', name="torso", type="box", size="0.5 0.25 0.25", pos="0 0 0.25", rgba="0 0 1 1", mass="0.1")

    # create an empty array for all wing parts to actuate later
    wing_list = []

    #---------------------------------------------------------
    # PARENT PARAMS
    range_list_right = []
    gear_list_right = []
    num_wings_right = 0
    print("PARENT PARAMS")

    tree = ET.parse('most_fit_parent3.xml')
    root = tree.getroot()
    # retrieve number of wings from most_fit_df.xml
    # THESE ARE NUM WINGS FROM PARENT
    for body in root.findall('.//body'):
        if body.get('name', '').startswith('wing'):
            num_wings_right += 1
    print("Number of wings found:", num_wings_right)  # Add this print statement

    # retrieve range of wing movement from most_fit_df.xml
    # THESE ARE RANGES FOR WINGS FROM PARENT
    for body in root.findall('.//joint'):
        if body.get('name', '').startswith('wing'):
            range_list_right.append(body.get('range'))
    print("Range of wings found:", range_list_right)  # Add this print statement

    # retrieve gear of wing movement from most_fit_df.xml
    # THESE ARE GEARS FOR WINGS FROM PARENT
    for body in root.findall('.//motor'):
        if body.get('name', '').startswith('wing'):
            gear_list_right.append(body.get('gear'))
    print("Gear of wings found:", gear_list_right)  # Add this print statement

    #---------------------------------------------------------
    # CHILD PARAMS
    range_list_left = []
    gear_list_left = []
    num_wings_left = 0
    print("CHILD PARAMS")

    # randomly generate number of wings for child
    num_wings_left = random.randint(1, 9)
    print(f"Number of wings: {num_wings_left}")

    # set range of wing 1 left to -1 1
    range_list_left.append("-1 1")
    # randomly generate range of wing movement for remaining child wings
    for i in range(num_wings_left-1):
        ran_range = random.randint(1, 100)
        range_list_left.append(f"{-ran_range} {ran_range}")
    print("Range of wings found:", range_list_left)  # Add this print statement

    # randomly generate gear for each wing
    for i in range(num_wings_left):
        ran_gear = random.randint(1, 100)
        gear_list_left.append(ran_gear)
    print("Gear of wings found:", gear_list_left)  # Add this print statement

    #---------------------------------------------------------
    # CREATE XML FILE
    # create elements/subelements
    mujoco = ET.Element('mujoco')

    worldbody = ET.SubElement(mujoco, 'worldbody')
    actuator = ET.SubElement(mujoco, 'actuator')

    # create plane and light
    light = ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
    plane = ET.SubElement(worldbody, 'geom', name="ground", type="plane", size="5 5 0.1", rgba=".9 .9 .9 1")

    # create organism body
    bf = ET.SubElement(worldbody, 'body', name="body")
    joint_free = ET.SubElement(bf, 'freejoint', name="root")
    pen_body = ET.SubElement(bf, 'geom', name="torso", type="box", size="0.5 0.25 0.25", pos="0 0 0.25", rgba="0 0 1 1", mass="0.1")

    # create winglists for parent and child
    wing_list_right = []
    wing_list_left = []

    # create and attach hinge joints for each number of num_wings_right
    for i in range(num_wings_right):
        # attach wing0 to body
        if i==0:
            wing = ET.SubElement(bf, 'body', name="wing" + str(i), pos="0 0.4 0.25", euler="0 0 90")
            joint_hinge = ET.SubElement(wing, 'joint', name="wing" + str(i) + "link", type="hinge", axis="0 0 1", range=range_list_right[i])
            wing_geom = ET.SubElement(wing, 'geom', name="wing" + str(i), type="cylinder", size="0.2 0.0325", pos="0 0 0", rgba="0 1 0 1", mass="1")
            # append wing to wing_list
            wing_list_right.append(wing)
            # print(f"wing_list: {wing_list}")
            continue 
        
        # attach each wing to the previous wing
        if i>0:
            wing = ET.SubElement(wing_list_right[i-1], 'body', name="wing" + str(i), euler="0 0 0")
            joint_hinge = ET.SubElement(wing_list_right[i-1], 'joint', name="wing" + str(i) + "link", type="hinge", pos="0 0 0", axis="1 0 0", range=range_list_right[i])
            wing_geom = ET.SubElement(wing_list_right[i-1], 'geom', name="wing" + str(i), type="cylinder", size="0.2 0.0325", pos=f"{0.4*i} 0 0", rgba="0 1 0 1", mass="1.5")
            wing_list_right.append(wing)
            # print(f"wing_list: {wing_list}")

    # create and attach hinge joints for each number of num_wings_left but on other side of body
    for i in range(num_wings_left):
        # attach wing0 to body
        if i==0:
            wing = ET.SubElement(bf, 'body', name="wingl" + str(i), pos="0 -0.4 0.25", euler="0 0 90")
            joint_hinge = ET.SubElement(wing, 'joint', name="wingl" + str(i) + "link", type="hinge", axis="0 0 1", range="-1 1")
            wing_geom = ET.SubElement(wing, 'geom', name="wingl" + str(i), type="cylinder", size="0.2 0.0325", pos="0 0 0", rgba="0 1 0 1", mass="1")
            # append wing to wing_list
            wing_list_left.append(wing)
            # print(f"wing_list: {wing_list}")
            continue 
        
        # attach each wing to the previous wing
        if i>0:
            wingl = ET.SubElement(wing_list_left[i-1], 'body', name="wingl" + str(i), euler="0 0 0")
            joint_hinge = ET.SubElement(wing_list_left[i-1], 'joint', name="wingl" + str(i) + "link", type="hinge", pos="0 0 0", axis="1 0 0", range=range_list_left[i])
            wing_geom = ET.SubElement(wing_list_left[i-1], 'geom', name="wingl" + str(i), type="cylinder", size="0.2 0.0325", pos=f"{-0.4*i} 0 0", rgba="0 1 0 1", mass="1.5")
            wing_list_left.append(wingl)
            # print(f"wing_list: {wing_list}")

    # create actuator for each wing in wing_list_right and wing_list_left
    for i in range(num_wings_right):
        motor = ET.SubElement(actuator, 'motor', name="wing" + str(i), joint="wing" + str(i) + "link", gear=str(gear_list_right[i]))
    for i in range(num_wings_left):
        motor = ET.SubElement(actuator, 'motor', name="wingl" + str(i), joint="wingl" + str(i) + "link", gear=str(gear_list_left[i]))


    # write to file
    # write to file
    with open('offspring.xml', 'wb') as file:
        tree = ET.ElementTree(mujoco)
        tree.write(file)
    # print("Offspring XML file created!")  # Add this print statement

    #---------------------------------------------------------
    # CALCULATE FITNESS
    model = mj.MjModel.from_xml_path('offspring.xml')
    data = mj.MjData(model)

    # create viewer
    viewer = mujoco_viewer.MujocoViewer(model, data)
    viewer.cam.distance = 15
    viewer.cam.azimuth = 60

    # set frequency and amplitude
    f = 0.1
    amp = 1.5

    tot_wings = num_wings_left + num_wings_right

    # Simulation loop
    for k in range(500):
        if viewer.is_alive:
            mj.mj_step(model, data)
            sin_wave = amp * math.sin(2 * math.pi * f * k)
            # Adjust loop iteration to match the correct number of wings
            for i in range(min(tot_wings, len(data.ctrl))):
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

    # average gear of organism
    gear_list_left_int = [int(gear) for gear in gear_list_left]
    g_left = sum(gear_list_left_int)/len(gear_list_left)
    gear_list_right_int = [int(gear) for gear in gear_list_right]
    g_right = sum(gear_list_right_int)/len(gear_list_right)
    avg_gear = (g_left+g_right)/2
    print(f"Average gear: {avg_gear}")

    # Function to calculate average range considering only positive values
    def calculate_avg_range(range_list):
        if not range_list:
            return 0

        range_list_values = [list(map(int, range_str.split())) for range_str in range_list]
        positive_ranges = [[val for val in range_vals if val > 0] for range_vals in range_list_values]
        avg_positive_ranges = [sum(range_vals) / len(range_vals) for range_vals in positive_ranges if range_vals]
        
        if avg_positive_ranges:
            return sum(avg_positive_ranges) / len(avg_positive_ranges)
        else:
            return 0

    # Calculate average range for left and right wings separately considering only positive values
    avg_range_left = calculate_avg_range(range_list_left)
    avg_range_right = calculate_avg_range(range_list_right)

    # Calculate overall average range
    avg_range = (avg_range_left + avg_range_right) / 2
    print(f"Average range: {avg_range}")

    # calculate overall fitness
    dist_fitness = dist_travelled/(num_wings_left+num_wings_right) * (avg_gear+avg_range)
    height_fitness = height_gained/(num_wings_left+num_wings_right) * (avg_gear+avg_range)
    fitness = dist_fitness + height_fitness
    print(f"Overall fitness: {fitness}")

    run_num_list.append(run+1)
    overall_fitness_list.append(fitness)
    dist_travelled_list.append(dist_travelled)
    height_gained_list.append(height_gained)
    avg_gear_list.append(avg_gear)
    avg_range_list.append(avg_range)

    print("---------------------------------------------------------")

    # Check if the current organism is the most fit
    if fitness > most_fit_fitness:
        # save this xml
        tree.write('most_fit_child3.xml')
        most_fit_fitness = fitness
        most_fit_params = {
            "num_wings": num_wings_left+num_wings_right,
            "range_list": range_list_left.copy() + range_list_right.copy(),
            "gear_list": gear_list_left.copy() + gear_list_right.copy()
        }
    
    # Close the viewer
    viewer.close()
        
# plot fitness of each run
plt.plot(run_num_list, overall_fitness_list, color='blue', marker='o', label='Fitness Points')
plt.plot(run_num_list, overall_fitness_list, color='green', linestyle='-', marker='', label='Fitness Line')
plt.xlabel('Organism Number')
plt.ylabel('Overall Fitness')
plt.title('Fitness of Dynamically Generated Children from Best Parent Round 3')

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
ax.set_title('Comparison of Average Values to Most Fit Child Round 3')
ax.set_xticks([i + bar_width / 2 for i in index])
ax.set_xticklabels(parameters)
ax.legend()

plt.show()
