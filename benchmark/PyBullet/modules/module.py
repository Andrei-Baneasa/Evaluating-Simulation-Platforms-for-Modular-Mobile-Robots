import pybullet as p
from modules.controllers.pid import PID as pid
from config import timestep
import matplotlib.pyplot as plt
from collections import deque

_module_index = 0
wheel_joint_indices = {0, 3}

class Module:
    def __init__(self, urdf_path, start_pos=(0, 0, 0.1), start_euler=(0, 0, 0), fixed_base=False):
        self.urdf_path = urdf_path
        self.start_pos = start_pos
        self.start_euler = start_euler
        # Convert Euler angles to quaternion for PyBullet
        self.start_ori = p.getQuaternionFromEuler(self.start_euler)        
        self.fixed_base = fixed_base

        self.pitch_error_history = deque(maxlen=200)
        self.control_signal_history = deque(maxlen=200)
        self.wheelspeed_history = deque(maxlen=200)
        self._setup_plot()

        self.robot_id = p.loadURDF(self.urdf_path,
                                   basePosition=self.start_pos,
                                   baseOrientation=self.start_ori,
                                   useFixedBase=self.fixed_base)
        
    def _setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label="Pitch Error")
        self.line2, = self.ax.plot([], [], label="Control Signal")
        self.line3, = self.ax.plot([], [], label="Wheel Speed")
        self.ax.legend()
        self.ax.set_xlabel("Step")
        self.ax.set_ylabel("Value")
        self.ax.set_title("Real-Time Pitch Error, Control Signal & Wheel Speed")

    def _update_plot(self):
        self.line1.set_data(range(len(self.pitch_error_history)), list(self.pitch_error_history))
        self.line2.set_data(range(len(self.control_signal_history)), list(self.control_signal_history))
        self.line3.set_data(range(len(self.wheelspeed_history)), list(self.wheelspeed_history))
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.001)

    def get_id(self):
        return self.robot_id
    
    def get_state(self):
        _, orn = p.getBasePositionAndOrientation(self.robot_id) # only need orientation (like from an IMU)
        euler = p.getEulerFromQuaternion(orn) #turn to euler's angles from quaternions

        wheel_speeds = [] #get wheel velocities    
        for joint_index in wheel_joint_indices:
            joint_state = p.getJointState(self.robot_id, joint_index)
            wheel_velocity = joint_state[1]  # [1] = joint velocity
            wheel_speeds.append(wheel_velocity)

        return {
            "orientation_quat": orn,
            "orientation_euler": euler,
            "wheel_speeds": wheel_speeds
        }    


    def _apply_wheel_velocity(self, joint_index, velocity):
        p.setJointMotorControl2(bodyIndex=self.robot_id,
                                jointIndex=joint_index,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=velocity)
        
    # def controller_step(self, target_velocity=0.0, target_orientation=0.0):
    #     state = self.get_state()
    #     current_orientation = state["orientation_euler"]
    #     pitch_error = -current_orientation[1]
    #     pid_controller = pid(9000, 1, 1, timestep)

    #     # Calculate control signal for each wheel
    #     for idx, joint_index in enumerate(wheel_joint_indices):
    #         control_signal = pid_controller.update(pitch_error)
    #         self._apply_wheel_velocity(joint_index, control_signal)
    #         # Store for plotting
    #         # self.pitch_error_history.append(pitch_error*1000)
    #         # self.control_signal_history.append(control_signal)
    #         # # Plot the first wheel's speed (or average if you prefer)
    #         # self.wheelspeed_history.append(state["wheel_speeds"][0])  # Assuming you want to plot the speed of the first wheel
    #         # self._update_plot()
    #         print(current_orientation[1])

    def controller_step(self, target_velocity=0.0, target_orientation=0.0):
        state = self.get_state()
        pitch = state["orientation_euler"][1]
        pitch_error = -pitch  # keep upright

        # --- PID term (same on both wheels to push/hold the body) ---
        u_pid = self.pid_controller.update(pitch_error)

        # --- RL term (stub) ---
        obs = self._build_obs(state)
        u_rl_L, u_rl_R = self._rl_action(obs)

        # --- Blend PID + RL (alpha = rl_weight) ---
        alpha = self.rl_weight
        u_L = (1 - alpha) * u_pid + alpha * u_rl_L
        u_R = (1 - alpha) * u_pid + alpha * u_rl_R

    def set_rl_policy(self, policy, weight=0.3):
        """policy must expose .act(obs)-> np.array([wL, wR]) in rad/s."""
        self.rl_policy = policy
        self.rl_weight = float(max(0.0, min(1.0, weight)))

    def _build_obs(self, state):
        e = state["orientation_euler"]   # roll, pitch, yaw
        wl, wr = state["wheel_speeds"]
        return [e[1], e[0], e[2], wl, wr]  # [pitch, roll, yaw, wL, wR]

    def _rl_action(self, obs):
        if self.rl_policy is None:
            return 0.0, 0.0
        a = self.rl_policy.act(obs)      # expect shape (2,)
        return float(a[0]), float(a[1])

    # def reset_if_tipped(self):
    #     pos, orn = p.getBasePositionAndOrientation(self.robot_id)
    #     euler = p.getEulerFromQuaternion(orn)
    #     if abs(euler[0]) > 0.78 or abs(euler[1]) > 0.78:
    #         p.resetBasePositionAndOrientation(self.robot_id, self.start_pos, self.start_ori)import pybullet as p
from modules.controllers.pid import PID as pid
from config import timestep
import matplotlib.pyplot as plt
from collections import deque

_module_index = 0
wheel_joint_indices = {0, 3}

class Module:
    def __init__(self, urdf_path, start_pos=(0, 0, 0.1), start_euler=(0, 0, 0), fixed_base=False):
        self.urdf_path = urdf_path
        self.start_pos = start_pos
        self.start_euler = start_euler
        # Convert Euler angles to quaternion for PyBullet
        self.start_ori = p.getQuaternionFromEuler(self.start_euler)        
        self.fixed_base = fixed_base

        self.pitch_error_history = deque(maxlen=200)
        self.control_signal_history = deque(maxlen=200)
        self.wheelspeed_history = deque(maxlen=200)
        self._setup_plot()

        self.robot_id = p.loadURDF(self.urdf_path,
                                   basePosition=self.start_pos,
                                   baseOrientation=self.start_ori,
                                   useFixedBase=self.fixed_base)
        
    def _setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label="Pitch Error")
        self.line2, = self.ax.plot([], [], label="Control Signal")
        self.line3, = self.ax.plot([], [], label="Wheel Speed")
        self.ax.legend()
        self.ax.set_xlabel("Step")
        self.ax.set_ylabel("Value")
        self.ax.set_title("Real-Time Pitch Error, Control Signal & Wheel Speed")

    def _update_plot(self):
        self.line1.set_data(range(len(self.pitch_error_history)), list(self.pitch_error_history))
        self.line2.set_data(range(len(self.control_signal_history)), list(self.control_signal_history))
        self.line3.set_data(range(len(self.wheelspeed_history)), list(self.wheelspeed_history))
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.001)

    def get_id(self):
        return self.robot_id
    
    def get_state(self):
        _, orn = p.getBasePositionAndOrientation(self.robot_id) # only need orientation (like from an IMU)
        euler = p.getEulerFromQuaternion(orn) #turn to euler's angles from quaternions

        wheel_speeds = [] #get wheel velocities    
        for joint_index in wheel_joint_indices:
            joint_state = p.getJointState(self.robot_id, joint_index)
            wheel_velocity = joint_state[1]  # [1] = joint velocity
            wheel_speeds.append(wheel_velocity)

        return {
            "orientation_quat": orn,
            "orientation_euler": euler,
            "wheel_speeds": wheel_speeds
        }    


    def _apply_wheel_velocity(self, joint_index, velocity):
        p.setJointMotorControl2(bodyIndex=self.robot_id,
                                jointIndex=joint_index,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=velocity)
        
    # def controller_step(self, target_velocity=0.0, target_orientation=0.0):
    #     state = self.get_state()
    #     current_orientation = state["orientation_euler"]
    #     pitch_error = -current_orientation[1]
    #     pid_controller = pid(9000, 1, 1, timestep)

    #     # Calculate control signal for each wheel
    #     for idx, joint_index in enumerate(wheel_joint_indices):
    #         control_signal = pid_controller.update(pitch_error)
    #         self._apply_wheel_velocity(joint_index, control_signal)
    #         # Store for plotting
    #         # self.pitch_error_history.append(pitch_error*1000)
    #         # self.control_signal_history.append(control_signal)
    #         # # Plot the first wheel's speed (or average if you prefer)
    #         # self.wheelspeed_history.append(state["wheel_speeds"][0])  # Assuming you want to plot the speed of the first wheel
    #         # self._update_plot()
    #         print(current_orientation[1])

    def controller_step(self, target_velocity=0.0, target_orientation=0.0):
        state = self.get_state()
        pitch = state["orientation_euler"][1]
        pitch_error = -pitch  # keep upright

        # --- PID term (same on both wheels to push/hold the body) ---
        u_pid = self.pid_controller.update(pitch_error)

        # --- RL term (stub) ---
        obs = self._build_obs(state)
        u_rl_L, u_rl_R = self._rl_action(obs)

        # --- Blend PID + RL (alpha = rl_weight) ---
        alpha = self.rl_weight
        u_L = (1 - alpha) * u_pid + alpha * u_rl_L
        u_R = (1 - alpha) * u_pid + alpha * u_rl_R

     def set_rl_policy(self, policy, weight=0.3):
        """policy must expose .act(obs)-> np.array([wL, wR]) in rad/s."""
        self.rl_policy = policy
        self.rl_weight = float(max(0.0, min(1.0, weight)))

     def _build_obs(self, state):
        e = state["orientation_euler"]   # roll, pitch, yaw
        wl, wr = state["wheel_speeds"]
        return [e[1], e[0], e[2], wl, wr]  # [pitch, roll, yaw, wL, wR]

    def _rl_action(self, obs):
        if self.rl_policy is None:
            return 0.0, 0.0
        a = self.rl_policy.act(obs)      # expect shape (2,)
        return float(a[0]), float(a[1])

    # def reset_if_tipped(self):
    #     pos, orn = p.getBasePositionAndOrientation(self.robot_id)
    #     euler = p.getEulerFromQuaternion(orn)
    #     if abs(euler[0]) > 0.78 or abs(euler[1]) > 0.78:
    #         p.resetBasePositionAndOrientation(self.robot_id, self.start_pos, self.start_ori)
