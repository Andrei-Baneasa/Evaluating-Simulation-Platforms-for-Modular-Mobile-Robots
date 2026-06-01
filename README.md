# Evaluating Simulation Platforms for Modular Mobile Robotic Systems

This repository contains the implementation, benchmark scenarios, and supporting material used in the study:

**"Evaluating Simulation Platforms for Modular Mobile Robotic Systems"**

The work investigates the suitability of three simulation environments for the development of **Modular Mobile Robotic Systems (MMRS)** composed of autonomous **Two-Wheel Inverted Pendulum (TWIP)** modules. The primary focus is on evaluating simulation workflows that support:

- Modular robot modeling
- Runtime constraint creation
- Dynamic reconfiguration
- Distributed control architectures
- Reinforcement learning integration
- Scalable multi-module simulations

---

## Repository Structure

The repository is organized into separate benchmark implementations for each evaluated simulation platform.

```text
.
├── benchmark/
│   ├── Webots/
│   ├── PyBullet/
│   └── Pinocchio/
│
├── README.md
```

Each simulation environment contains its own implementation, dependencies, setup instructions, and benchmark-specific documentation.

---

## Simulation Platforms

### Webots

Webots was evaluated as a complete robotics simulation environment featuring:

- Integrated graphical interface
- Built-in physics simulation
- Connector-based modular robot assembly
- Supervisor controllers
- ROS integration

For installation instructions, usage examples, and benchmark details see:

➡️ **[benchmark/Webots/README.md](benchmark/Webots/README.md)**

---

### PyBullet

PyBullet was evaluated as a flexible, programmatic simulation environment suitable for:

- Constraint-based modular robot modeling
- Runtime docking and reconfiguration
- Reinforcement learning workflows
- Headless execution
- Parallel simulation

For installation instructions, usage examples, and benchmark details see:

➡️ **[benchmark/PyBullet/README.md](benchmark/PyBullet/README.md)**

---

### Pinocchio + Jiminy

Pinocchio and Jiminy were evaluated as a control-oriented simulation workflow emphasizing:

- Rigid-body dynamics
- Analytical modeling
- Control algorithm development
- State estimation
- Optimization-oriented robotics research

Due to current software compatibility requirements, the benchmark implementation was developed and tested under Linux / WSL2.

For installation instructions, usage examples, and benchmark details see:

➡️ **[benchmark/Pinocchio/README.md](benchmark/Pinocchio/README.md)**

---

## Benchmark Philosophy

The repository follows a common modeling strategy across all platforms:

1. Each TWIP module is modeled as an independent robot.
2. Modules are instantiated separately.
3. Inter-module connections are represented through simulation constraints or equivalent mechanisms.
4. Control and commands are executed independently for each module.
5. Performance metrics are collected during simulation execution.

The objective is not to identify a universally superior simulator, but rather to evaluate how naturally each platform supports the requirements of Modular Mobile Robotic Systems.

---

## Benchmark Metrics

The evaluated benchmark scenarios focus on:

- CPU utilization
- Memory consumption
- Runtime performance
- Ease of implementation
- Constraint handling
- Modularity support
- Reinforcement learning compatibility

The benchmark implementations intentionally remain simple and reproducible to allow straightforward comparison between platforms.

---

## Running the Benchmarks

Each simulator has different dependencies and execution procedures.

Please follow the platform-specific instructions:

| Platform | Documentation |
|-----------|---------------|
| Webots | `benchmark/Webots/README.md` |
| PyBullet | `benchmark/PyBullet/README.md` |
| Pinocchio + Jiminy | `benchmark/Pinocchio/README.md` |

---

## Paper

The repository accompanies the research paper:

**Evaluating Simulation Platforms for Modular Mobile Robotic Systems**

The study compares Webots, Pinocchio/Jiminy, and PyBullet from the perspective of modular robotic systems that require:

- Runtime topology changes
- Distributed control
- Dynamic module coupling
- Reinforcement learning integration

---

## Citation

If you use this repository in academic work, please cite the associated publication.

```bibtex
@article{baneasa2026simulation,
  title={Evaluating Simulation Platforms for Modular Mobile Robotic Systems},
  author={Baneasa, Andrei and Buleandra, Debora-Gabriela and others},
  journal={Machines},
  year={2026}
}
```

---

## License

This project is released under the license included in the repository.

---

## Author

**Andrei Baneasa**  
Technical University of Cluj-Napoca  
Faculty of Automotive, Mechatronics and Mechanical Engineering

GitHub: https://github.com/Andrei-Baneasa
