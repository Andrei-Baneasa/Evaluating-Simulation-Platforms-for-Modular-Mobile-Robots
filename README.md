# Evaluating-Simulation-Platforms-for-Modular-Mobile-Robots
This repository contains the code and other sources for the paper Evaluating Simulation Platforms for Modular Mobile Robots and Developing a Scalable Architecture for Reinforcement Learning.
The structure is as follows:
  * The benchmark folder contains the benchmarks for WeBots and Pinocchio. See respective folders for instructions
  * The modular_mobile_robot submodule contains the PyBullet simulation. Its benchmark was obtained by changing the following:
    * Using the line configuration, one of the modules was removed and in the enviroment setup the GUI was disabled
    * To run, just install PyBullet, NumPy and several other packages using pip or conda
  * sources: pictures, sources for diagrams, etc.
