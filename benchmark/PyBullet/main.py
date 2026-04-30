import pybullet as p
import time
import math
import pybullet_data
from modules.module import Module
from environment import setup_environment
from config import ROBOT_URDF_PATH, timestep
from robot.robot import Robot

def main():
    #create desired environment
    setup_environment()

    # module1 = Module(ROBOT_URDF_PATH)
    robot1 = Robot(module_path=[ROBOT_URDF_PATH],configuration="line")

    step_count = 0
    while True:
        p.stepSimulation()
       
        time.sleep(timestep)

if __name__ == "__main__":
    main()

