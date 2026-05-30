import argparse
import math
import time

import pybullet as p

from environment import setup_environment
from config import ROBOT_URDF_PATH, timestep


WHEEL_SPEED = 8.0
WHEEL_FORCE = 2.0
REPORT_EVERY = 120

# These are the same connector link indices already used in robot.py
# robot.py connects connector_link_1 of module 0 to connector_link_2 of module 1.
PARENT_CONNECTOR_LINK = 1
CHILD_CONNECTOR_LINK = 2


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Disable PyBullet GUI")
    parser.add_argument("--steps", type=int, default=1000, help="Total simulation steps")
    parser.add_argument("--dock-step", type=int, default=250, help="Step at which the modules dock")
    parser.add_argument("--undock-step", type=int, default=700, help="Step at which the modules undock")
    parser.add_argument("--no-sleep", action="store_true", help="Run as fast as possible instead of sleeping between GUI steps")
    parser.add_argument("--connector-max-force", type=float, default=500.0, help="Maximum force used by the docking constraint")
    return parser.parse_args()


def find_wheel_joints(body_id):
    wheel_joints = []

    for joint_id in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_id)
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        joint_type = info[2]

        if joint_type == p.JOINT_REVOLUTE:
            if "wheel" in joint_name.lower() or "wheel" in link_name.lower():
                wheel_joints.append(joint_id)

    return wheel_joints


def set_wheel_speed(body_id, speed, force=WHEEL_FORCE):
    wheel_joints = find_wheel_joints(body_id)
    print(f"Body {body_id} wheel joints: {wheel_joints}")

    for joint_id in wheel_joints:
        p.setJointMotorControl2(
            bodyUniqueId=body_id,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=speed,
            force=force,
        )


def stop_wheels(body_id):
    for joint_id in find_wheel_joints(body_id):
        p.setJointMotorControl2(
            bodyUniqueId=body_id,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0.0,
            force=WHEEL_FORCE,
        )


def get_link_pose(body_id, link_id):
    state = p.getLinkState(body_id, link_id, computeForwardKinematics=True)
    return state[4], state[5]  # worldLinkFramePosition, worldLinkFrameOrientation


def get_base_pose(body_id):
    return p.getBasePositionAndOrientation(body_id)


def transform_between(base_pose, link_pose):
    """Return transform from base frame to link frame."""
    base_pos, base_orn = base_pose
    link_pos, link_orn = link_pose
    inv_base_pos, inv_base_orn = p.invertTransform(base_pos, base_orn)
    return p.multiplyTransforms(inv_base_pos, inv_base_orn, link_pos, link_orn)


def align_child_connector_to_parent(parent_body, parent_link, child_body, child_link):
    """
    Move child_body so child_link exactly matches parent_link.

    This is a scripted alignment step. It avoids asking the wheel controller to solve
    the docking maneuver and isolates the thing we actually want to test here:
    runtime constraint creation/removal.
    """
    parent_link_pose = get_link_pose(parent_body, parent_link)
    child_base_pose = get_base_pose(child_body)
    child_link_pose = get_link_pose(child_body, child_link)

    base_to_child_link = transform_between(child_base_pose, child_link_pose)
    inv_base_to_child_link = p.invertTransform(*base_to_child_link)

    new_child_base_pose = p.multiplyTransforms(
        parent_link_pose[0], parent_link_pose[1],
        inv_base_to_child_link[0], inv_base_to_child_link[1]
    )

    p.resetBasePositionAndOrientation(child_body, new_child_base_pose[0], new_child_base_pose[1])
    p.resetBaseVelocity(child_body, [0, 0, 0], [0, 0, 0])


def connector_distance(body_a, link_a, body_b, link_b):
    pa, _ = get_link_pose(body_a, link_a)
    pb, _ = get_link_pose(body_b, link_b)
    return math.dist(pa, pb)


def load_module(start_pos, start_euler=(0.0, 0.0, 0.0)):
    return p.loadURDF(
        ROBOT_URDF_PATH,
        basePosition=start_pos,
        baseOrientation=p.getQuaternionFromEuler(start_euler),
        useFixedBase=False,
    )


def main():
    args = parse_args()

    setup_environment(gui=not args.headless)
    p.setTimeStep(timestep)
    p.setRealTimeSimulation(0)

    # Optional camera setup for GUI runs.
    if not args.headless:
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=35,
            cameraPitch=-35,
            cameraTargetPosition=[0.0, 0.0, 0.05],
        )

    # Two independent modules, initially separated along X.
    module_a = load_module(start_pos=[+0.04, 0.0, 0.10])
    module_b = load_module(start_pos=[-0.04, 0.0, 0.10])

    print("\n=== Runtime Docking / Undocking Test ===")
    print(f"module_a body id: {module_a}")
    print(f"module_b body id: {module_b}")
    print(f"dock_step: {args.dock_step}")
    print(f"undock_step: {args.undock_step}")
    print(f"total steps: {args.steps}")
    print(f"timestep: {timestep:.6f} s")

    # Move both modules gently before docking. The exact motion is not the point;
    # the topology change is created by createConstraint/removeConstraint below.
    set_wheel_speed(module_a, +WHEEL_SPEED)
    set_wheel_speed(module_b, +WHEEL_SPEED)

    constraint_id = None
    docked = False
    start_wall_time = time.perf_counter()

    for step in range(args.steps):
        if step == args.dock_step and not docked:
            # Scripted connector alignment before creating the physical constraint.
            align_child_connector_to_parent(
                parent_body=module_a,
                parent_link=PARENT_CONNECTOR_LINK,
                child_body=module_b,
                child_link=CHILD_CONNECTOR_LINK,
            )

            before_dist = connector_distance(
                module_a, PARENT_CONNECTOR_LINK,
                module_b, CHILD_CONNECTOR_LINK,
            )

            constraint_id = p.createConstraint(
                parentBodyUniqueId=module_a,
                parentLinkIndex=PARENT_CONNECTOR_LINK,
                childBodyUniqueId=module_b,
                childLinkIndex=CHILD_CONNECTOR_LINK,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0],
            )

            p.changeConstraint(constraint_id, maxForce=args.connector_max_force)
            docked = True

            print(
                f"DOCK step={step}, "
                f"constraint_id={constraint_id}, "
                f"connector_distance_before={before_dist:.6f} m"
            )

            if not args.headless:
                p.addUserDebugText(
                    "DOCK",
                    textPosition=[0, 0.15, 0.25],
                    textColorRGB=[0, 0, 1],
                    lifeTime=2.0,
                )

        if step == args.undock_step and docked:
            p.removeConstraint(constraint_id)
            print(f"UNDOCK step={step}, removed_constraint_id={constraint_id}")
            constraint_id = None
            docked = False

            # After undocking, push module_b slightly away so the separation is visible.
            p.resetBaseVelocity(module_b, linearVelocity=[0.20, 0.0, 0.0], angularVelocity=[0, 0, 0])

            if not args.headless:
                p.addUserDebugText(
                    "UNDOCK",
                    textPosition=[0, -0.15, 0.25],
                    textColorRGB=[1, 0, 0],
                    lifeTime=2.0,
                )

        p.stepSimulation()

        if step % REPORT_EVERY == 0 or step in (args.dock_step, args.undock_step):
            sim_time = step * timestep
            wall_time = time.perf_counter() - start_wall_time
            rtf = sim_time / wall_time if wall_time > 0 else 0.0

            pos_a, _ = p.getBasePositionAndOrientation(module_a)
            pos_b, _ = p.getBasePositionAndOrientation(module_b)
            dist_base = math.dist(pos_a, pos_b)
            dist_conn = connector_distance(
                module_a, PARENT_CONNECTOR_LINK,
                module_b, CHILD_CONNECTOR_LINK,
            )

            print(
                f"step={step:04d}, sim_time={sim_time:.3f}s, "
                f"RTF={rtf:.2f}, docked={docked}, "
                f"base_dist={dist_base:.4f} m, connector_dist={dist_conn:.6f} m"
            )

        if not args.headless and not args.no_sleep:
            time.sleep(timestep)

    total_wall_time = time.perf_counter() - start_wall_time
    total_sim_time = args.steps * timestep

    print("\n=== Runtime Docking / Undocking Results ===")
    print(f"steps: {args.steps}")
    print(f"simulated_time_s: {total_sim_time:.6f}")
    print(f"wall_time_s: {total_wall_time:.6f}")
    print(f"real_time_factor: {total_sim_time / total_wall_time:.6f}")
    print(f"dock_step: {args.dock_step}")
    print(f"undock_step: {args.undock_step}")
    print("success: docking constraint was created and removed during the active simulation loop")

    p.disconnect()


if __name__ == "__main__":
    main()
