# PyBullet Benchmark Implementation

## Overview

This directory contains the PyBullet implementation used to evaluate simulation strategies for Modular Mobile Robotic Systems (MMRS) composed of Two-Wheel Inverted Pendulum (TWIP) modules.

PyBullet was selected due to its support for runtime constraint creation, headless execution, reinforcement learning integration, and flexible programmatic control of independently simulated robotic modules.

Each module is represented as an independent rigid body loaded from a URDF model. Inter-module connections are created using runtime constraints, enabling the simulation of dynamically reconfigurable robotic structures.

The implementation was designed to evaluate:

* Runtime performance
* Memory consumption
* CPU utilization
* Dynamic constraint creation and removal
* Suitability for modular robotic systems
* Compatibility with reinforcement learning workflows

## Requirements

Install the required Python packages:

```bash
pip install -r requirements.txt
```

## Running the Benchmark

### Standard Benchmark

Run the baseline benchmark scenario:

```bash
python main.py
```

The benchmark loads two TWIP modules connected through a fixed constraint and executes the simulation for a predefined number of steps while collecting performance metrics.

### Headless Execution

For performance measurements, it is recommended to disable rendering:

```bash
python main.py --headless
```

Headless execution removes graphical overhead and provides more consistent timing measurements.

## Dynamic Docking Benchmark

A supplementary benchmark demonstrating runtime topology modification is provided.

### Graphical Mode

```bash
python main_docking.py
```

### Headless Mode

```bash
python main_docking.py --headless
```

This scenario was introduced to evaluate one of the key requirements of Modular Mobile Robotic Systems: dynamic reconfiguration.

The benchmark consists of:

1. Two independent modules.
2. Runtime creation of a fixed constraint (docking event).
3. Motion while connected.
4. Runtime removal of the constraint (undocking event).

Unlike the baseline benchmark, the docking scenario explicitly demonstrates runtime modification of the simulated robot topology through constraint creation and removal during execution.

## Robot Model

The robot model is based on a simplified Two-Wheel Inverted Pendulum (TWIP) platform represented using URDF.

Each module is simulated independently and may be connected to other modules through PyBullet constraints. This approach enables flexible experimentation with modular robot topologies without requiring modifications to the underlying robot description.

## Repository Structure

```text
PyBullet/
├── main.py
├── main_docking.py
├── requirements.txt
├── modules/
├── urdf/
└── assets/
```

## Notes

The implementation prioritizes flexibility, reproducibility, and support for modular robotic experimentation rather than absolute physical accuracy.

The benchmark scenarios were intentionally kept simple in order to isolate the effects of constraint handling, runtime performance, and dynamic reconfiguration workflows.

The provided examples are intended as reference implementations for evaluating simulation workflows and can be extended to larger modular configurations, decentralized control architectures, and reinforcement learning applications.
