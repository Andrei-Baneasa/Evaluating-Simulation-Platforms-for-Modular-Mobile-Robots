import pybullet as p
import time
import math
import pybullet_data
import argparse

from environment import setup_environment
from config import ROBOT_URDF_PATH, timestep
from robot.robot import Robot


WHEEL_SPEED = 8.0       # rad/s
WHEEL_FORCE = 2.0       # motor max force
SIM_DURATION = 10.0     # seconds
REPORT_EVERY = 240      # 240 steps = 1 second at 1/240



def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable PyBullet GUI"
    )
    return parser.parse_args()

    return parser.parse_args()

def find_wheel_joints(body_id):
    wheel_joints = []

    for j in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, j)

        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        joint_type = info[2]

        if joint_type == p.JOINT_REVOLUTE:
            if "wheel" in joint_name.lower() or "wheel" in link_name.lower():
                wheel_joints.append(j)

    return wheel_joints


def set_forward_motion(robot):
    for module in robot.modules:
        body_id = module.robot_id
        wheel_joints = find_wheel_joints(body_id)

        print(f"Body {body_id} wheel joints:", wheel_joints)

        for joint_id in wheel_joints:
            p.setJointMotorControl2(
                bodyUniqueId=body_id,
                jointIndex=joint_id,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=WHEEL_SPEED,
                force=WHEEL_FORCE
            )


def get_robot_center(robot):
    positions = []

    for module in robot.modules:
        pos, orn = p.getBasePositionAndOrientation(module.robot_id)
        positions.append(pos)

    x = sum(pos[0] for pos in positions) / len(positions)
    y = sum(pos[1] for pos in positions) / len(positions)
    z = sum(pos[2] for pos in positions) / len(positions)

    return [x, y, z]


def main():
    
    args = parse_args()

    setup_environment(gui=not args.headless)

    p.setTimeStep(timestep)
    p.setRealTimeSimulation(0)

    robot1 = Robot(
        module_path=[ROBOT_URDF_PATH],
        configuration="line"
    )

    set_forward_motion(robot1)

    start_pos = get_robot_center(robot1)
    start_wall_time = time.perf_counter()

    step_count = 0
    # max_steps = int(SIM_DURATION / timestep)
    max_steps = 1000
    
    print("\n=== PyBullet Benchmark Start ===")
    print(f"Simulation duration target: {SIM_DURATION:.3f} s")
    print(f"Timestep: {timestep:.6f} s")
    print(f"Max steps: {max_steps}")
    print(f"Initial robot center: {start_pos}")

    while step_count < max_steps:
        p.stepSimulation()
        step_count += 1

        if step_count % REPORT_EVERY == 0:
            sim_time = step_count * timestep
            wall_time = time.perf_counter() - start_wall_time
            real_time_factor = sim_time / wall_time if wall_time > 0 else 0.0

            current_pos = get_robot_center(robot1)

            dx = current_pos[0] - start_pos[0]
            dy = current_pos[1] - start_pos[1]
            dz = current_pos[2] - start_pos[2]
            distance_xy = math.sqrt(dx * dx + dy * dy)

            print(
                f"step={step_count}, "
                f"sim_time={sim_time:.3f}s, "
                f"wall_time={wall_time:.3f}s, "
                f"RTF={real_time_factor:.3f}, "
                f"x={current_pos[0]:.4f}, "
                f"y={current_pos[1]:.4f}, "
                f"z={current_pos[2]:.4f}, "
                f"dist_xy={distance_xy:.4f}"
            )

        time.sleep(timestep)

    end_wall_time = time.perf_counter()
    total_wall_time = end_wall_time - start_wall_time
    total_sim_time = step_count * timestep
    final_pos = get_robot_center(robot1)

    dx = final_pos[0] - start_pos[0]
    dy = final_pos[1] - start_pos[1]
    dz = final_pos[2] - start_pos[2]
    distance_xy = math.sqrt(dx * dx + dy * dy)

    print("\n=== PyBullet Benchmark Results ===")
    print(f"steps: {step_count}")
    print(f"simulated_time_s: {total_sim_time:.6f}")
    print(f"wall_time_s: {total_wall_time:.6f}")
    print(f"real_time_factor: {total_sim_time / total_wall_time:.6f}")
    print(f"initial_position: {start_pos}")
    print(f"final_position: {final_pos}")
    print(f"dx: {dx:.6f}")
    print(f"dy: {dy:.6f}")
    print(f"dz: {dz:.6f}")
    print(f"distance_xy: {distance_xy:.6f}")


if __name__ == "__main__":
    main()
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

