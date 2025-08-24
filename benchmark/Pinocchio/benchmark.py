
from os.path import dirname, join, abspath
import numpy as np
import time
import jiminy_py.core as jiminy
from jiminy_py.simulator import Simulator
from jiminy_py.viewer import Viewer
import meshcat
import pinocchio as pin
import sys

print("Python version:", sys.version)

# Get the directory where the script is located
script_dir = dirname(abspath(__file__))
# Define the base path for the models
model_base_dir = join(script_dir, "models", "twip2")

# Define paths for URDF and mesh directories
urdf_dir = join(model_base_dir, "urdf")
mesh_dir = join(model_base_dir, "meshes")

# Specify the URDF file
urdf_filename = "twip.SLDASM.urdf"  # Change this if the filename is different
urdf_path = join(urdf_dir, urdf_filename)

 # Instantiate the robots
 # ROBOT 1
robot1 = jiminy.Robot("robot1")
robot1.initialize(urdf_path, has_freeflyer=True)
constraint_left = jiminy.WheelConstraint(
        "wheel_1", 0.0225, np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
robot1.add_constraint("wheel_1", constraint_left)
constraint_left.baumgarte_freq = 20.0
constraint_right = jiminy.WheelConstraint(
        "wheel_2", 0.0225, np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
robot1.add_constraint("wheel_2", constraint_right)
constraint_right.baumgarte_freq = 20.0

  # ROBOT 2
robot2 = jiminy.Robot("robot2")
robot2.initialize(urdf_path, has_freeflyer=True)
constraint_left2 = jiminy.WheelConstraint(
        "wheel_1", 0.0225, np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
robot2.add_constraint("wheel_1", constraint_left2)
constraint_left2.baumgarte_freq = 20.0
constraint_right2 = jiminy.WheelConstraint(
        "wheel_2", 0.0225, np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
robot2.add_constraint("wheel_2", constraint_right2)
constraint_right2.baumgarte_freq = 20.0


    # Instantiate a multi-robot engine
engine = jiminy.Engine()
engine.add_robot(robot1)
engine.add_robot(robot2)

# Define coupling force
stiffness = np.array([0.2, 0.5, 1.0, 0.2, 0.3, 0.6])
damping = np.array([0.0, 0.7, 0.3, 0.5, 0.8, 1.1])
alpha = 0.5
engine.register_viscoelastic_coupling_force(
        "robot1", "robot2", "base_link", "base_link",
        stiffness, damping, alpha)
q_init, v_init = {}, {}


q1 = pin.neutral(robot1.pinocchio_model)   # 9 elements
v1 = np.zeros(robot1.nv) 
q_init["robot1"] = q1
v_init["robot1"] = v1
q2 = pin.neutral(robot2.pinocchio_model)   # 9 elements
v2 = np.zeros(robot2.nv)
q_init["robot2"] = q2
v_init["robot2"] = v2

# Suppose wheel_1 and wheel_2 are revolute joints of robot1
v_init["robot1"][robot1.pinocchio_model.getJointId("wheel_1")] = 2.0  # rad/s
v_init["robot1"][robot1.pinocchio_model.getJointId("wheel_2")] = 2.0  # rad/s

v_init["robot2"][robot2.pinocchio_model.getJointId("wheel_1")] = 2.0  # rad/s
v_init["robot2"][robot2.pinocchio_model.getJointId("wheel_2")] = 2.0  # rad/s
engine.start(q_init, v_init)

import time
import psutil

process = psutil.Process()
cpu_usage = []
mem_usage = []
n_steps = 1000
start_time = time.time()

for step in range(n_steps):
    engine.step()
    # sample usage
    cpu_usage.append(psutil.cpu_percent(interval=0.01, percpu=True))  # per core
    mem_usage.append(process.memory_info().rss / (1024**2))  # MB

end_time = time.time()
duration = end_time - start_time

# Report
max_cpu_per_core = np.max(cpu_usage, axis=0)
max_ram_mb = np.max(mem_usage)

print(f"Ran {n_steps} steps in {duration:.2f} seconds")
print(f"Max CPU usage per core: {max_cpu_per_core}")
print(f"Max RAM usage (MB): {max_ram_mb:.2f}")
print(f"Total CPU usage: {psutil.cpu_percent(interval=1)}%")
print("DONE")