# Evaluating-Simulation-Platforms-for-Modular-Mobile-Robots
This repository contains the code and other sources for the paper Evaluating Simulation Platforms for Modular Mobile Robots and Developing a Scalable Architecture for Reinforcement Learning.
The structure is as follows:
  * The benchmark folder contains the benchmarks for WeBots and Pinocchio. See respective folders for instructions
  * The modular_mobile_robot submodule contains the PyBullet simulation. Its benchmark was obtained by changing the following:
    * Using the line configuration, one of the modules was removed and in the enviroment setup the GUI was disabled
    * To run, just install PyBullet, NumPy and several other packages using pip or conda
    * future development will not be pushed to this repo. It may or may not become public at a later date
**Disclaimer** Due to time constaints, each benchmark uses a different version of a differentially driven robot. In WeBots, an existing example was modified to fit the goal. In Pinocchio, an older URDF file created by the authors was used. In PyBullet, a different placeholder URDF was used. Since the goal was to evaluate how these platforms perform, we feel that the actual robot model use does not have a notable impact on the testing.
  * sources: pictures, sources for diagrams, etc.
