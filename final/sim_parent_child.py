import mujoco
import mujoco_viewer
import numpy as np
import dm_control.mujoco
import math
import mujoco as mj
import xml.etree.ElementTree as ET

# Function to get the number of wings from XML file
def get_num_wings(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    num_wings = 0
    # Traverse the XML tree to find body elements with names starting with 'wing'
    for body in root.findall('.//body'):
        if body.get('name', '').startswith('wing'):
            num_wings += 1
    print("Number of wings found:", num_wings)  # Add this print statement
    return num_wings

# Function to simulate the model
def simulate(model_path):
    num_wings = get_num_wings(model_path)  # Get the number of wings dynamically
    model = mj.MjModel.from_xml_path(model_path)
    data = mj.MjData(model)
    
    # Initialize control array with the appropriate size
    data.ctrl = np.zeros((num_wings,))  # Make it a 1D array of size num_wings

    viewer = mujoco_viewer.MujocoViewer(model, data)
    viewer.cam.distance = 15
    viewer.cam.azimuth = 60
    f = 0.1
    amp = 1.5

    for k in range(500):
        if viewer.is_alive:
            mj.mj_step(model, data)
            sin_wave = amp * math.sin(2 * math.pi * f * k)
            # Set control inputs for each wing
            for i in range(num_wings):
                data.ctrl[i] = sin_wave
                # print("Setting control input for wing", i, "to", sin_wave)  # Add this print statement
            viewer.render()
        else:
            break

    viewer.close()

# Path to the XML files
parent_xml_path = 'most_fit_parent3.xml'
child_xml_path = 'most_fit_child3.xml'

# Simulate parent
print("SIMULATING PARENT!")
simulate(parent_xml_path)

# Simulate child
print("SIMULATING CHILD!")
simulate(child_xml_path)