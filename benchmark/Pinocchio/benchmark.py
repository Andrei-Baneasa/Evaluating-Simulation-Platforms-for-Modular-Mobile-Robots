from os.path import dirname, join, abspath
import time
import numpy as np
import jiminy_py.core as jiminy
from jiminy_py.viewer import Viewer
import pinocchio as pin
import psutil
import re

# ============================================================
# Enable/Disable visualisation using Meshcat
# ============================================================
ENABLE_VISUALIZATION = False
# ============================================================
# Paths / URDF fixup
# ============================================================

script_dir = dirname(abspath(__file__))
model_base_dir = join(script_dir, "models", "twip2")
urdf_dir = join(model_base_dir, "urdf")
mesh_dir = join(model_base_dir, "meshes")

urdf_model_path = join(urdf_dir, "twip.SLDASM.urdf")
fixed_urdf_model_path = join(urdf_dir, "twip_fixed.urdf")

with open(urdf_model_path, "r") as f:
    urdf_content = f.read()

mesh_dir_linux = abspath(mesh_dir).replace("\\", "/")

# Replace any absolute SolidWorks-exported mesh path with the local mesh folder.
mesh_dir_linux = abspath(mesh_dir).replace("\\", "/")

# Replace any absolute SolidWorks-exported mesh path with the local mesh folder.
urdf_content = re.sub(
    r'filename="[^"]*/meshes/([^"/]+\.STL)"',
    rf'filename="{mesh_dir_linux}/\1"',
    urdf_content
)

urdf_content = re.sub(
    r'filename="[^"]*\\meshes\\([^"\\]+\.STL)"',
    rf'filename="{mesh_dir_linux}/\1"',
    urdf_content
)

# Same contact-frame trick as the working one-robot script.
contact_frames_xml = """
  <link name="wheel_1_contact"/>
  <joint name="wheel_1_contact_joint" type="fixed">
    <origin xyz="0.0072647 0 0" rpy="0 0 0"/>
    <parent link="wheel_1"/>
    <child link="wheel_1_contact"/>
  </joint>

  <link name="wheel_2_contact"/>
  <joint name="wheel_2_contact_joint" type="fixed">
    <origin xyz="-0.073867 0 0" rpy="0 0 0"/>
    <parent link="wheel_2"/>
    <child link="wheel_2_contact"/>
  </joint>
"""

if "wheel_1_contact" not in urdf_content:
    urdf_content = urdf_content.replace("</robot>", contact_frames_xml + "\n</robot>")

with open(fixed_urdf_model_path, "w") as f:
    f.write(urdf_content)

urdf_model_path = fixed_urdf_model_path


# ============================================================
# Settings
# ============================================================

wheel_radius = 0.0285
ground_normal = np.array([0.0, 0.0, 1.0])
wheel_axis = np.array([1.0, 0.0, 0.0])

# Initial side-by-side spacing. This is also the relaxed distance of the coupling.
robot2_y_offset = 0.70

# Viscoelastic 6D coupling. Units are roughly translational/rotational stiffness + damping.
# Increase these if they drift apart too much. Decrease if the simulation gets angry.
stiffness = np.array([0.0, 0.65, 0.0, 0.0, 0.0, 0.0])
damping   = np.array([0.0, 0.10, 0.0, 0.0, 0.0, 0.0])
alpha = 0.5

n_steps = 1000
visualize_every = 5

controller_calls = {"robot1": 0}

# ============================================================
# Robot construction
# ============================================================
def make_robot(name):
    robot = jiminy.Robot(name)
    robot.initialize(
        urdf_model_path,
        mesh_package_dirs=[mesh_dir],
        has_freeflyer=True
    )

    for wheel_name in ["wheel_1_contact", "wheel_2_contact"]:
        c = jiminy.WheelConstraint(
            wheel_name,
            wheel_radius,
            ground_normal,
            wheel_axis
        )
        robot.add_constraint(wheel_name, c)
        c.baumgarte_freq = 2.0

    motor_1 = jiminy.SimpleMotor("joint_1")
    robot.attach_motor(motor_1)
    motor_1.initialize("joint_1")

    motor_2 = jiminy.SimpleMotor("joint_2")
    robot.attach_motor(motor_2)
    motor_2.initialize("joint_2")

    
    def constant_torque_controller(t, q, v, sensors_data, command):
        controller_calls["robot1"] += 1
        command[:] = 0.0
        command[0] = 5.0
        command[1] = 5.0

    robot.controller = jiminy.FunctionalController(constant_torque_controller)

    return robot

def put_wheels_on_ground(robot, q):
    model = robot.pinocchio_model
    data = model.createData()

    v_zero = np.zeros(robot.nv)
    pin.forwardKinematics(model, data, q, v_zero)
    pin.updateFramePlacements(model, data)

    z1 = data.oMf[model.getFrameId("wheel_1_contact")].translation[2]
    z2 = data.oMf[model.getFrameId("wheel_2_contact")].translation[2]

    q[2] += wheel_radius - min(z1, z2)
    return q


def set_joint_velocity(robot, v, joint_name, value):
    jid = robot.pinocchio_model.getJointId(joint_name)
    idx = robot.pinocchio_model.joints[jid].idx_v
    v[idx] = value


robot1 = make_robot("robot1")
robot2 = make_robot("robot2")

print("robot1 controller:", robot1.controller)
print("robot1 nmotors:", robot1.nmotors)
print("robot1 command fields:", robot1.log_command_fieldnames)
# ============================================================
# Initial state dictionaries for the multi-robot Engine
# ============================================================

q_init = {}
v_init = {}

q1 = pin.neutral(robot1.pinocchio_model)
v1 = np.zeros(robot1.nv)
q1 = put_wheels_on_ground(robot1, q1)

q2 = pin.neutral(robot2.pinocchio_model)
v2 = np.zeros(robot2.nv)
q2[0] = -0.07
q2[1] = 0.0
q2 = put_wheels_on_ground(robot2, q2)

# Give both robots the same initial wheel spin so it is obvious they are linked.
set_joint_velocity(robot1, v1, "joint_1", 2.0)
set_joint_velocity(robot1, v1, "joint_2", 2.0)
set_joint_velocity(robot2, v2, "joint_1", 2.0)
set_joint_velocity(robot2, v2, "joint_2", 2.0)

q_init["robot1"] = q1
v_init["robot1"] = v1
q_init["robot2"] = q2
v_init["robot2"] = v2

print("robot1 q[:3]:", q1[:3])
print("robot2 q[:3]:", q2[:3])
print("initial separation:", q2[:3] - q1[:3])


# ============================================================
# Multi-robot engine + physical coupling
# ============================================================

engine = jiminy.Engine()
engine.add_robot(robot1)
engine.add_robot(robot2)

engine_options = engine.get_options()
engine_options["constraints"]["regularization"] = 0.2
engine_options["contacts"]["model"] = "constraint"
engine_options["stepper"]["odeSolver"] = "runge_kutta_dopri"
engine_options["stepper"]["dtMax"] = 1.0e-2
engine.set_options(engine_options)

# This is the important line: a compliant 6D link between the two base frames.
# Your older script had this exact idea, but it was commented out.
engine.register_viscoelastic_coupling_force(
    "robot1", "robot2",
    "connector_left", "connector_right",
    stiffness, damping, alpha
)

engine.start(q_init, v_init)


# ============================================================
# Visualization (optional)
# ============================================================

viewer1 = None
viewer2 = None

if ENABLE_VISUALIZATION:

    viewer1 = Viewer(robot1, backend="meshcat", robot_name="robot1")
    viewer2 = Viewer(robot2, backend="meshcat", robot_name="robot2")

    viewer1.display(q_init["robot1"])
    viewer2.display(q_init["robot2"])

    print("\nOpen Meshcat manually if it did not open:")
    print("http://127.0.0.1:7000/static/")
    input("Press ENTER once Meshcat is open...")


# ============================================================
# Step simulation and display both states together
# ============================================================

process = psutil.Process()
cpu_usage = []
cpu_samples = []
mem_usage = []
start_time = time.time()
# Prime psutil
process.cpu_percent(interval=None)

for step in range(n_steps):
    engine.step()

    if step % 50 == 0:
            cpu_samples.append(
                process.cpu_percent(interval=None)
            )
            
    if ENABLE_VISUALIZATION and step % visualize_every == 0:
        states = engine.robot_states
        viewer1.display(states[0].q)
        viewer2.display(states[1].q)

        if step % 500 == 0:
            p1 = states[0].q[:3]
            p2 = states[1].q[:3]
            print(
                f"step {step:05d} | "
                f"p1={p1} | p2={p2} | dist={np.linalg.norm(p2 - p1):.4f}"
            )
        
    # cpu_usage.append(psutil.cpu_percent(interval=0.01, percpu=True))  # per core
    cpu_usage.append(psutil.cpu_percent(interval=None, percpu=True))
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

print(f"Average process CPU usage: {np.mean(cpu_samples):.2f}%")
print(f"Peak process CPU usage: {np.max(cpu_samples):.2f}%")

print("DONE")
