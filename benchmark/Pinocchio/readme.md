# Pinocchio / Jiminy MMRS Benchmark

Constraint-based simulation benchmark for Modular Mobile Robotic Systems (MMRS) using:

- Pinocchio
- Jiminy
- Meshcat (Optional)

The benchmark simulates multiple independently modeled TWIP (Two-Wheel Inverted Pendulum) modules connected at runtime using compliant constraints.

---

# Features

- Runtime inter-module constraints
- Multi-robot simulation
- Shared Meshcat visualization
- Headless benchmark mode
- CPU / RAM profiling
- Automatic URDF mesh path correction
- Portable repository structure

---

# Repository Structure

```text
benchmark/Pinocchio/
├── benchmark.py
├── requirements.txt
└── models/
    └── twip2/
        ├── urdf/
        │   ├── twip.SLDASM.urdf
        │   └── twip_fixed.urdf
        └── meshes/
            ├── base_link.STL
            ├── wheel_1.STL
            └── wheel_2.STL
```

---

# Recommended Environment

Tested with:

- Ubuntu 22.04
- WSL2
- Python 3.12

WSL2 is recommended on Winodws for Jiminy / Pinocchio compatibility.

---

# Installation

Create and activate a virtual environment:

```bash
python3 -m venv jiminy-env
source jiminy-env/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

# Running

```bash
python benchmark.py
```

---

# Visualization

Toggle visualization inside `benchmark.py`:

```python
ENABLE_VISUALIZATION = True
```

or:

```python
ENABLE_VISUALIZATION = False
```

## Visualization Enabled

- launches Meshcat
- displays both robots live
- useful for debugging

Open manually if needed:

```text
http://127.0.0.1:7000/static/
```

## Visualization Disabled

- fully headless
- no rendering overhead
- recommended for benchmarking

---

# Benchmark

Default configuration:

- 2 coupled TWIP modules
- 1000 simulation steps
- 10 ms timestep
- 10 seconds simulated time

The script reports:

- runtime
- RAM usage
- CPU usage
- module separation distance

---

# Runtime Constraints

Modules are connected dynamically using:

```python
engine.register_viscoelastic_coupling_force(...)
```

This allows runtime modular composition and reconfiguration without rebuilding the robot model.

---

# Automatic URDF Mesh Fixing

The script automatically converts SolidWorks-exported absolute mesh paths into local repository-relative paths.

No manual URDF editing should be required after cloning the repository.

---

# Main Dependencies

```text
numpy>=2.2.6
psutil>=7.2.2
meshcat>=0.3.2

pin>=4.0.0
libpinocchio>=4.0.0
eigenpy>=3.13.0

jiminy-py>=1.8.13

trimesh>=4.12.2

Panda3D>=1.10.16
panda3d-viewer>=0.4.1
panda3d-simplepbr>=0.11.2
```

