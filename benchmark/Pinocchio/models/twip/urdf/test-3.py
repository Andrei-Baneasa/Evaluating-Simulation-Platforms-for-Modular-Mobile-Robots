import pinocchio as pin
import numpy as np
import time
from os.path import dirname, join, abspath
from pinocchio.visualize import MeshcatVisualizer

# Get script directory
script_dir = dirname(abspath(__file__))

# Define paths for URDF and mesh directories
model_base_dir = join(script_dir, "models", "twip")
urdf_dir = join(model_base_dir, "urdf")
mesh_dir = join(model_base_dir, "meshes")
urdf_filename = "twip.urdf"
urdf_model_path = join(urdf_dir, urdf_filename)

# Load the URDF model
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)
data = model.createData()

# Set gravity
model.gravity = pin.Motion(np.array([0, 0, -9.81, 0, 0, 0]))

# Initialize Meshcat visualizer
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

# Define initial state
q = pin.neutral(model)
v = np.zeros(model.nv)  # Joint velocities
tau = np.zeros(model.nv)  # Joint torques
time_step = 0.01  # Integration step

# Define wheel radius (change if different)
wheel_radius = 0.1  # Example value, adjust to match your model

# Get wheel joint IDs
left_wheel_joint_id = model.getJointId("joint-1")
right_wheel_joint_id = model.getJointId("joint-2")
print("Left wheel joint ID:", left_wheel_joint_id)
print("Right wheel joint ID:", right_wheel_joint_id)

# Get chassis joint ID
chassis_joint_id = model.getJointId("universe")
print("Chassis joint ID:", chassis_joint_id)

# Create contact placement for each wheel on the ground
ground_normal = np.array([0, 0, 1])  # z-axis is up
ground_pos = np.array([0, 0, 0])     # ground at z=0

# Create the constraint models for wheel-ground rolling contacts
constraint_models = []
for joint_id in [left_wheel_joint_id, right_wheel_joint_id]:
    contact_point = np.array([0, 0, -wheel_radius])  # Contact point at bottom of wheel
    wheel_contact = pin.RigidConstraintModel(
        pin.ContactType.ROLLING_CONTACT, 
        model, 
        joint_id, 
        pin.SE3(np.eye(3), contact_point),
        pin.LOCAL_WORLD_ALIGNED,
        ground_normal
    )
    constraint_models.append(wheel_contact)

# Create constraint data
constraint_data = [pin.RigidConstraintData(cm) for cm in constraint_models]

# Compute the constraint Jacobian
def compute_constraint_jacobian(model, data, q, constraint_models):
    J = np.zeros((sum(cm.size() for cm in constraint_models), model.nv))
    pin.computeAllTerms(model, data, q, v)
    
    row = 0
    for cm in constraint_models:
        J_i = pin.getConstraintJacobian(model, data, cm, q)
        size = cm.size()
        J[row:row+size] = J_i
        row += size
    
    return J

# Function to update robot state with rolling constraint
def update_state_with_constraints(model, data, q, v, tau, constraint_models, constraint_data, dt):
    # Compute dynamics terms
    pin.computeAllTerms(model, data, q, v)
    
    # Get constraint Jacobian
    J = compute_constraint_jacobian(model, data, q, constraint_models)
    
    # Compute M and h terms from articulated body algorithm
    M = data.M                             # Mass matrix
    h = data.nle                           # Nonlinear effects
    
    # Formulate constrained dynamics problem: M*a + h = tau + J^T*lambda
    # Solve for joint accelerations and constraint forces
    JMinv = J @ np.linalg.inv(M)
    lambda_c = np.linalg.solve(JMinv @ J.T, JMinv @ (tau - h))
    
    # Compute joint accelerations
    a = np.linalg.solve(M, tau - h + J.T @ lambda_c)
    
    # Integrate
    v_next = v + a * dt
    q_next = pin.integrate(model, q, v_next * dt)
    
    return q_next, v_next

# Simulation loop
step = 0
duration = 10.0  # Simulate for 10 seconds
simulation_time = 0.0

while simulation_time < duration:
    # Apply torque to wheels to move forward
    tau[0] = 0.5  # Apply torque to left wheel
    tau[1] = 0.5  # Apply torque to right wheel
    
    # Update state considering rolling constraints
    q, v = update_state_with_constraints(model, data, q, v, tau, constraint_models, constraint_data, time_step)
    
    # Update forward kinematics
    pin.forwardKinematics(model, data, q)
    
    # Visualization
    viz.display(q)
    
    # Sleep to maintain real-time factor
    time.sleep(time_step)
    
    # Update simulation time
    simulation_time += time_step
    step += 1
    
    # Print status every second
    if step % 100 == 0:
        print(f"Time: {simulation_time:.2f}s, Position: {q[0:3]}")